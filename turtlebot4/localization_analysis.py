"""Assess bounded localization evidence separately from AMCL lifecycle/readiness."""
import math

from turtlebot4.localization_health import (
    MAX_POSITION_STD_M, MAX_YAW_STD_RAD, transform_pose, validate_registration,
    require_reference_scope,
)


def assess_convergence(samples, registration, minimum_seconds=5.0):
    """Require sustained uncertainty, fresh sensing, geometry and reference accuracy.

    Inputs are timestamped measurements from the simulation observer. This does
    not force AMCL updates, resample particles, or rewrite covariance.
    """
    transform = validate_registration(registration)
    if len(samples) < 2:
        raise ValueError('Insufficient convergence observations')
    times = [s['sim_time'] for s in samples]
    progressing = all(b > a for a, b in zip(times, times[1:]))
    errors, headings, bounds = [], [], []
    for sample in samples:
        reference = transform_pose(sample['world_pose'], transform)
        require_reference_scope(*reference[:2], registration)
        error = math.dist(reference[:2], sample['map_pose'][:2])
        errors.append(error)
        headings.append(abs(math.remainder(reference[2]-sample['map_pose'][2], 2*math.pi)))
        bounds.append(error + registration['reference_uncertainty_m'])
    covariance = all(s['covariance']['acceptable']
                     and s['covariance']['position_major_std_m'] <= MAX_POSITION_STD_M
                     and s['covariance']['yaw_std_rad'] <= MAX_YAW_STD_RAD for s in samples)
    fresh = all(abs(s['tf_age']) <= 1.5 and abs(s['truth_age']) <= .10
                and all(-.10 <= age <= 1. for age in s['ages'].values()) for s in samples)
    geometry = all(s['scan_alignment']['p90_m'] <= .15
                   and s['scan_alignment']['fraction_within_0_15m'] >= .95 for s in samples)
    # Compare map-frame motion with independently measured world motion, so
    # ordinary translation cannot masquerade as an unstable localization jump.
    jumps = []
    for a, b in zip(samples, samples[1:]):
        ra, rb = transform_pose(a['world_pose'], transform), transform_pose(b['world_pose'], transform)
        jumps.append(math.hypot((b['map_pose'][0]-a['map_pose'][0])-(rb[0]-ra[0]),
                                (b['map_pose'][1]-a['map_pose'][1])-(rb[1]-ra[1])))
    checks = {'clock_progressing': progressing,
              'meaningful_observation': times[-1]-times[0] >= minimum_seconds,
              'covariance_acceptable': covariance, 'sensor_tf_fresh': fresh,
              'scan_map_agreement': geometry, 'no_unexplained_position_jumps': max(jumps) <= .15,
              'registered_accuracy': max(bounds) <= .30 and max(headings) <= .25}
    return {'pass': all(checks.values()), 'checks': checks,
            'duration_sec': times[-1]-times[0], 'samples': len(samples),
            'max_position_residual_m': max(errors), 'max_position_error_bound_m': max(bounds),
            'max_yaw_residual_rad': max(headings), 'max_jump_residual_m': max(jumps),
            'ground_truth_accuracy_verified': checks['registered_accuracy'],
            'reference_uncertainty_m': registration['reference_uncertainty_m']}


def laser_endpoints(scan, laser_to_base):
    """Apply the measured planar extrinsic to valid native LaserScan endpoints."""
    from turtlebot4.localization_health import yaw
    if laser_to_base.header.frame_id != 'base_link' or laser_to_base.child_frame_id != scan.header.frame_id:
        raise ValueError('Laser extrinsic frames disagree with scan/base')
    t = laser_to_base.transform
    result = []
    for i, distance in enumerate(scan.ranges):
        if math.isfinite(distance) and scan.range_min < distance < scan.range_max:
            angle = scan.angle_min + i * scan.angle_increment + yaw(t.rotation)
            result.append((t.translation.x + distance*math.cos(angle),
                           t.translation.y + distance*math.sin(angle)))
    return result


def particle_spread(cloud):
    """Describe emitted particle weights; these are usually post-resampling weights."""
    import numpy as np
    from scipy.ndimage import gaussian_filter1d
    from scipy.signal import find_peaks
    if not cloud.particles:
        raise ValueError('Empty particle cloud')
    xy = np.array([[p.pose.position.x,p.pose.position.y] for p in cloud.particles])
    weights = np.array([p.weight for p in cloud.particles])
    if not np.all(np.isfinite(xy)) or not np.all(np.isfinite(weights)) or np.any(weights < 0) or weights.sum() <= 0:
        raise ValueError('Invalid particle positions/weights')
    weights /= weights.sum()
    mean = np.sum(xy*weights[:,None],axis=0)
    centered = xy-mean
    eigenvalues, directions = np.linalg.eigh((centered*weights[:,None]).T@centered)
    projected = centered@directions[:,-1]
    histogram, _ = np.histogram(projected,bins=40,weights=weights)
    density = gaussian_filter1d(histogram,1.5)
    peaks, _ = find_peaks(np.pad(density,1),prominence=max(density)*.20)
    return {'count':len(weights), 'post_resample_effective_count':float(1/np.sum(weights**2)),
            'major_position_sigma_m':float(np.sqrt(max(0,eigenvalues[-1]))),
            'major_axis_density_modes':len(peaks), 'mean_xy':mean.tolist(),
            'pre_resampling_effective_count_observable':False}
