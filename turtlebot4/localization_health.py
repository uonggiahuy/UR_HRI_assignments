"""Shared uncertainty and timestamp policy; covariance is not physical accuracy."""
import math

from turtlebot4.diagnostics import seconds

MAX_POSITION_STD_M = 0.30
MAX_YAW_STD_RAD = math.radians(15.0)
MAX_SENSOR_AGE_SEC = 1.0
MAX_POSE_AGE_SEC = 2.0
MAX_FUTURE_SKEW_SEC = 0.10
STATIONARY_OBSERVATION_SEC = 2.0


def yaw(quaternion):
    q = quaternion
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def transform_pose(pose, transform):
    """Apply target<-source SE(2) to an (x,y,yaw) source-frame pose."""
    x, y, heading = pose
    tx, ty, angle = transform
    if not all(math.isfinite(v) for v in (*pose, *transform)):
        raise ValueError('Nonfinite planar pose/transform')
    c, s = math.cos(angle), math.sin(angle)
    return (tx + c * x - s * y, ty + s * x + c * y,
            math.remainder(heading + angle, 2 * math.pi))


def covariance_health(msg):
    """Bound the largest planar covariance eigenvalue, including correlation."""
    cov = msg.pose.covariance
    indices = (0, 1, 5)
    matrix = [[float(cov[i * 6 + j]) for j in indices] for i in indices]
    finite = all(math.isfinite(value) for row in matrix for value in row)
    symmetric = all(abs(matrix[i][j] - matrix[j][i]) <= 1e-8
                    for i in range(3) for j in range(3))
    # A covariance must be positive semidefinite, including all principal minors.
    a, b, c = matrix[0]; _, d, e = matrix[1]; _, _, f = matrix[2]
    determinant = a * d * f + 2 * b * c * e - a * e * e - d * c * c - f * b * b
    psd = (finite and symmetric and min(a, d, f) >= 0
           and min(a*d-b*b, a*f-c*c, d*f-e*e, determinant) >= -1e-10)
    major = (a + d + math.hypot(a - d, 2 * b)) / 2 if finite else math.inf
    accepted = psd and major <= MAX_POSITION_STD_M**2 and f <= MAX_YAW_STD_RAD**2
    return {'acceptable': accepted, 'positive_semidefinite': psd,
            'position_major_std_m': math.sqrt(max(0, major)),
            'yaw_std_rad': math.sqrt(max(0, f)) if finite else math.inf,
            'limits': {'position_major_std_m': MAX_POSITION_STD_M,
                       'yaw_std_rad': MAX_YAW_STD_RAD}}


def stamp_fresh(stamp, now, max_age=MAX_SENSOR_AGE_SEC):
    age = now - seconds(stamp)
    return math.isfinite(now) and seconds(stamp) > 0 and -MAX_FUTURE_SKEW_SEC <= age <= max_age


def pose_time_health(msg, now, scan, odom, tf_pose, stationary_observed=False):
    """Old retained AMCL output needs stationary evidence and current TF agreement.

    Receipt time is deliberately excluded: transient-local delivery does not
    change measurement time. Moving localization requires a recent estimate.
    """
    age = now - seconds(msg.header.stamp)
    support = (scan is not None and odom is not None
               and stamp_fresh(scan.header.stamp, now) and stamp_fresh(odom.header.stamp, now))
    p = msg.pose.pose
    agrees = (math.hypot(p.position.x - tf_pose[0], p.position.y - tf_pose[1]) <= .15
              and abs(math.remainder(yaw(p.orientation) - tf_pose[2], 2*math.pi)) <= .15)
    stopped = (odom is not None and abs(odom.twist.twist.linear.x) < .02
               and abs(odom.twist.twist.linear.y) < .02 and abs(odom.twist.twist.angular.z) < .05)
    retained = age > MAX_POSE_AGE_SEC
    valid = (support and seconds(msg.header.stamp) > 0 and age >= -MAX_FUTURE_SKEW_SEC
             and (not retained or (stationary_observed and stopped and agrees)))
    return {'acceptable': valid, 'age_sec': age, 'retained_stationary': retained and valid,
            'sensor_freshness_pass': support, 'pose_tf_agreement': agrees}


def validate_registration(registration):
    """Require a reviewed landmark registration; metadata alone is not evidence.

    The offline artifact must identify source/target frames and independently
    measured non-collinear correspondences. Never derive this from AMCL output.
    """
    if not registration or registration.get('verified') is not True:
        raise ValueError('Physical accuracy requires independently verified world-to-map registration')
    if registration.get('map_frame') != 'map' or not registration.get('world_frame'):
        raise ValueError('Registration source/target frames are missing or invalid')
    transform = (*registration['translation'], registration['yaw'])
    if len(transform) != 3 or not all(math.isfinite(v) for v in transform):
        raise ValueError('Invalid registration transform')
    source, target = registration['world_points'], registration['map_points']
    if len(source) < 3 or len(source) != len(target):
        raise ValueError('Require at least three independent corresponding landmarks')
    a, b = source[:2]
    if not any(abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])) > 1e-4 for c in source[2:]):
        raise ValueError('Registration landmarks are collinear')
    uncertainty = registration['reference_uncertainty_m']
    if not math.isfinite(uncertainty) or not 0 < uncertainty <= .15:
        raise ValueError('Registration reference uncertainty exceeds 0.15 m')
    for world, mapped in zip(source, target):
        predicted = transform_pose((*world, 0.), transform)
        if not all(math.isfinite(v) for v in mapped) or math.dist(predicted[:2], mapped) > uncertainty:
            raise ValueError('Landmark residual exceeds reference uncertainty')
    bounds = registration.get('scope_map_bounds')
    if bounds is not None:
        if (len(bounds) != 4 or not all(math.isfinite(v) for v in bounds)
                or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]):
            raise ValueError('Invalid registration scope')
    for check in registration.get('heldout_scans', []):
        if not math.isfinite(check['max_m']) or check['max_m'] > uncertainty:
            raise ValueError('Held-out registration validation exceeds uncertainty')
    return transform


def require_reference_scope(x, y, registration):
    bounds = registration.get('scope_map_bounds')
    if bounds and not (bounds[0] <= x <= bounds[1] and bounds[2] <= y <= bounds[3]):
        raise ValueError('Coordinate outside validated reference scope')


def validate_reference_for_map(registration, map_path):
    """Bind a reviewed reference to its exact immutable occupancy files."""
    import hashlib
    from pathlib import Path
    import yaml
    validate_registration(registration)
    expected = registration.get('map_sha256')
    if not expected:
        raise ValueError('Registration must identify the map SHA-256 hashes')
    path = Path(map_path)
    image = path.parent / yaml.safe_load(path.read_text())['image']
    actual = {name: hashlib.sha256(file.read_bytes()).hexdigest()
              for name, file in (('yaml', path), ('pgm', image))}
    if actual != expected:
        raise ValueError('Registration map hashes differ from the current map')


def aligned_truth(msg, registration):
    transform = validate_registration(registration)
    if msg.header.frame_id != registration['world_frame']:
        raise ValueError('Ground-truth frame differs from registered source frame')
    p = msg.pose.pose
    pose = transform_pose((p.position.x, p.position.y, yaw(p.orientation)), transform)
    require_reference_scope(*pose[:2], registration)
    return pose


def reference_accuracy(pose, truth, now, registration):
    """Simulation reference check; include uncertainty rather than covariance alone."""
    if truth is None or not stamp_fresh(truth.header.stamp, now, max_age=.10):
        raise ValueError('Ground truth absent or not synchronized within 0.10 s')
    reference = aligned_truth(truth, registration)
    error = math.dist(pose[:2], reference[:2])
    heading = abs(math.remainder(pose[2]-reference[2], 2*math.pi))
    bound = error + registration['reference_uncertainty_m']
    return {'verified': bound <= .30 and heading <= .25,
            'position_residual_m': error, 'position_error_bound_m': bound,
            'yaw_residual_rad': heading, 'aligned_truth': reference,
            'reference_uncertainty_m': registration['reference_uncertainty_m']}
