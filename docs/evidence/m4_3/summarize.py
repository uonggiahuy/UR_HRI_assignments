"""Recompute final-reference assessments and plots from preserved measurements.

Run from this directory in the sourced Humble/assignment/TF2 environment.
The user map is read only for the derived local navigation plot.
"""
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from turtlebot4.localization_analysis import assess_convergence, particle_spread
from nav2_msgs.msg import Particle, ParticleCloud


def main():
    root = Path(__file__).resolve().parent
    reference = json.loads((root / 'registration.json').read_text())
    assessments, trials = {}, {}
    for path in sorted(root.glob('*.json')):
        record = json.loads(path.read_text())
        if not isinstance(record, dict) or not record.get('samples') or 'after' not in record:
            continue
        try:
            assessments[path.stem] = assess_convergence(record['samples'], reference)
        except ValueError as exc:
            assessments[path.stem] = {'pass': False, 'error': str(exc)}
        final = record['after']
        trials[path.stem] = {
            'experiment_completed': record['status'], 'convergence': assessments[path.stem],
            'position_sigma_m': final['covariance']['position_major_std_m'],
            'yaw_sigma_rad': final['covariance']['yaw_std_rad'],
            'unique_amcl_updates': record.get('unique_amcl_updates'),
            'world_displacement_m': record.get('world_pose_displacement_m'),
            'world_yaw_change_rad': record.get('world_pose_yaw_change_rad'),
            'odom_displacement_m': record.get('odom_pose_displacement_m'),
            'odom_yaw_change_rad': record.get('odom_pose_yaw_change_rad'),
            'final_cloud': final.get('cloud'), 'bumper_contacts': record['bumper_contacts'],
            'hazards': record['hazards'], 'stopped_velocity': final['velocity']}
    (root / 'final_reference_assessments.json').write_text(json.dumps(trials, indent=2)+'\n')

    fig, axes = plt.subplots(2, 4, figsize=(15, 7), constrained_layout=True)
    experiments = [
        ('Baseline', ['stationary_baseline','rotation_baseline','translation_short_baseline','combined_baseline']),
        ('Motion noise candidate (rejected)', ['stationary_motion_candidate','rotation_motion_candidate','translation_motion_candidate','combined_motion_candidate']),
        ('Probabilistic laser only', ['stationary_sensor_candidate','rotation_sensor_candidate','translation_sensor_candidate','combined_sensor_candidate']),
        ('Shipped laser + TF correction', ['stationary_verified','rotation_verified','translation_verified',None])]
    for label, names in experiments:
        for column, name in enumerate(names):
            if name is None:
                continue
            samples = json.loads((root / (name+'.json')).read_text())['samples']
            elapsed = [s['sim_time']-samples[0]['sim_time'] for s in samples]
            axes[0,column].plot(elapsed,[s['covariance']['position_major_std_m'] for s in samples],label=label)
            axes[1,column].plot(elapsed,[math.degrees(s['covariance']['yaw_std_rad']) for s in samples])
    for column, title in enumerate(('Stationary','Rotation','Translation','Combined')):
        axes[0,column].set_title(title)
        axes[0,column].axhline(.30,color='black',linestyle='--')
        axes[1,column].axhline(15,color='black',linestyle='--')
        axes[1,column].set_xlabel('Seconds within independent trial')
        for row in range(2):axes[row,column].grid(alpha=.3)
    axes[0,0].set_ylabel('Major XY sigma (m)');axes[1,0].set_ylabel('Yaw sigma (degrees)')
    axes[0,0].legend(fontsize=7)
    fig.savefig(root/'covariance_comparison.png',dpi=160);plt.close(fig)

    particle_metrics = {}
    fig, axes = plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
    for ax, name in zip(axes,('rotation_baseline','rotation_verified')):
        particles = json.loads((root/(name+'.json')).read_text())['final_particles']
        cloud = ParticleCloud()
        for x,y,heading,weight in particles:
            p=Particle();p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(heading/2);p.pose.orientation.w=math.cos(heading/2)
            p.weight=weight;cloud.particles.append(p)
        particle_metrics[name]=particle_spread(cloud)
        xy=np.array(particles)[:,:2];ax.scatter(*xy.T,s=2,alpha=.4)
        ax.set_title(name);ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)')
        ax.set_aspect('equal');ax.grid(alpha=.3)
    fig.savefig(root/'particle_spread.png',dpi=160);plt.close(fig)
    (root/'particle_metrics.json').write_text(json.dumps(particle_metrics,indent=2)+'\n')

    goal = json.loads((root/'a_to_b.json').read_text())['goals'][0]
    physical_position = goal['physical_position_goal_error']
    physical_yaw = goal['physical_yaw_goal_error']
    acceptance = {
        'requirement': 'Exactly one A-to-B autonomous NavigateToPose demonstration',
        'nav2_execution_pass': goal['action_result']==4,
        'physical_arrival_position_residual_m': physical_position,
        'physical_arrival_position_error_interval_m': [max(0.,physical_position-reference['reference_uncertainty_m']),physical_position+reference['reference_uncertainty_m']],
        'physical_arrival_yaw_residual_rad': physical_yaw,
        'physical_arrival_yaw_error_bound_rad': physical_yaw+reference['reference_yaw_uncertainty_rad'],
        'configured_tolerances': goal['configured_tolerances'],
        'physical_arrival_within_tolerance_verified': False,
        'overall_m4_3': 'FAIL / physical arrival tolerance NOT VERIFIED',
        'note': 'Raw a_to_b.json PASS refers to original recorder execution checks. Final acceptance separately includes physical reference uncertainty. No action was repeated for this assessment.'}
    (root/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n')

    fig, ax = plt.subplots(figsize=(7,6),constrained_layout=True)
    image = np.flipud(np.array(Image.open(root.parents[2]/'maps/map.pgm')))
    ax.imshow(image,cmap='gray',origin='lower',extent=(-15.,15.15,-25.1,25.),vmin=0,vmax=255)
    points = np.array(goal['preflight_plan']['points']);ax.plot(*points.T,'b--',label='Navfn global path')
    observations = goal['motion_observations']
    ax.plot(*np.array([s['pose'][:2] for s in observations]).T,color='tab:orange',label='Localized trajectory')
    ax.plot(*np.array([s['truth'][:2] for s in observations]).T,color='tab:green',label='Independently registered truth')
    target = goal['goal'][:2];ax.plot(*target,'r*',markersize=13,label='B')
    ax.add_patch(plt.Circle(target,.25,fill=False,color='red',label='Configured 0.25 m goal tolerance'))
    ax.plot(*goal['start']['pose'][:2],'ko',label='Verified A')
    ax.plot(*goal['final']['truth'][:2],'gx',markersize=9)
    ax.set_xlim(-2.,.4);ax.set_ylim(-1.,.5);ax.set_aspect('equal');ax.grid(alpha=.3)
    ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.legend(fontsize=7)
    ax.set_title('One real A-to-B action: SUCCEEDED; physical arrival tolerance unverified')
    fig.savefig(root/'a_to_b_geometry.png',dpi=180);plt.close(fig)


if __name__ == '__main__':
    main()
