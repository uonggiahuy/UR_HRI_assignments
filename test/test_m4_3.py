"""M4.3 parameter, native sensor, convergence and reference regression contracts."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Particle, ParticleCloud
from sensor_msgs.msg import LaserScan
import pytest
import yaml

from turtlebot4.localization_analysis import assess_convergence, laser_endpoints, particle_spread
from turtlebot4.localization_health import (
    validate_registration, validate_reference_for_map, require_reference_scope,
)
from turtlebot4.localization_runner import default_localization_params_path, build_localization_launch_cmd

ROOT = Path(__file__).resolve().parents[1]


def reference():
    return {'verified':True, 'world_frame':'warehouse', 'map_frame':'map',
            'translation':[0.,0.], 'yaw':0., 'reference_uncertainty_m':.05,
            'world_points':[[0.,0.],[1.,0.],[0.,1.]],
            'map_points':[[0.,0.],[1.,0.],[0.,1.]], 'scope_map_bounds':[-4.,4.,-4.,2.],
            'heldout_scans':[{'max_m':.02}]}


def samples(moving=False):
    return [{'sim_time':float(i+1), 'world_pose':[i*.05 if moving else 0.,0.,0.],
             'map_pose':[i*.05 if moving else 0.,0.,0.],
             'covariance':{'acceptable':True,'position_major_std_m':.08,'yaw_std_rad':.02},
             'ages':{'scan':.02,'odom':.01},'truth_age':.01,'tf_age':.01,
             'scan_alignment':{'p90_m':.05,'fraction_within_0_15m':.99}} for i in range(11)]


def test_amcl_owned_parameters_change_only_evidenced_laser_and_tf_settings():
    actual=yaml.safe_load(default_localization_params_path().read_text())
    upstream=yaml.safe_load((Path(get_package_share_directory('turtlebot4_navigation'))/'config/localization.yaml').read_text())
    assert actual['amcl']['ros__parameters']['laser_model_type']=='likelihood_field_prob'
    assert actual['amcl']['ros__parameters']['transform_tolerance']==.2
    actual['amcl']['ros__parameters']['laser_model_type']='likelihood_field'
    actual['amcl']['ros__parameters']['transform_tolerance']=1.0
    assert actual==upstream
    assert f'params:={default_localization_params_path()}' in build_localization_launch_cmd()


@pytest.mark.parametrize('moving',[False,True])
def test_sustained_stationary_and_moving_convergence(moving):
    result=assess_convergence(samples(moving),reference())
    assert result['pass'] and result['ground_truth_accuracy_verified']
    assert result['max_jump_residual_m']==pytest.approx(0)


@pytest.mark.parametrize('fault', ['covariance','scan','tf','truth','geometry','jump','accuracy','clock'])
def test_small_covariance_alone_cannot_establish_convergence(fault):
    data=samples()
    if fault=='covariance':data[-1]['covariance']['position_major_std_m']=.31
    elif fault=='scan':data[-1]['ages']['scan']=2.
    elif fault=='tf':data[-1]['tf_age']=3.
    elif fault=='truth':data[-1]['truth_age']=.4
    elif fault=='geometry':data[-1]['scan_alignment']['fraction_within_0_15m']=.6
    elif fault=='jump':data[-1]['map_pose']=[.2,0.,0.]
    elif fault=='accuracy':
        for s in data:s['map_pose']=[.26,0.,0.]
    elif fault=='clock':data[-1]['sim_time']=data[-2]['sim_time']
    assert not assess_convergence(data,reference())['pass']


def test_covariance_reduction_needs_sustained_window_and_geometry():
    data=samples()
    data[0]['covariance']['acceptable']=False
    assert not assess_convergence(data,reference())['pass']
    assert not assess_convergence(samples()[:3],reference())['pass']


def test_native_lidar_mount_yaw_and_scan_frame():
    scan=LaserScan();scan.header.frame_id='laser';scan.range_min=.1;scan.range_max=20.
    scan.angle_min=0.;scan.angle_increment=math.pi/2;scan.ranges=[1.,2.,float('inf')]
    tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='laser'
    tf.transform.translation.x=-.04;tf.transform.rotation.z=math.sin(math.pi/4);tf.transform.rotation.w=math.cos(math.pi/4)
    points=laser_endpoints(scan,tf)
    assert points[0]==pytest.approx((-.04,1.)) and points[1]==pytest.approx((-2.04,0.))
    tf.child_frame_id='wrong'
    with pytest.raises(ValueError,match='frames'):laser_endpoints(scan,tf)


@pytest.mark.parametrize('fault',['collinear','residual','heldout','uncertainty','scope'])
def test_invalid_or_ambiguous_registration_rejected(fault):
    r=reference()
    if fault=='collinear':r['world_points'][2]=[2.,0.]
    elif fault=='residual':r['map_points'][2]=[.2,1.]
    elif fault=='heldout':r['heldout_scans'][0]['max_m']=.16
    elif fault=='uncertainty':r['reference_uncertainty_m']=.151
    elif fault=='scope':r['scope_map_bounds']=[1.,-1.,0.,2.]
    with pytest.raises(ValueError):validate_registration(r)


def test_reference_bound_to_unchanged_user_map_and_region(tmp_path):
    before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'maps').glob('map.*')}
    r=reference();r['map_sha256']={'yaml':before['map.yaml'],'pgm':before['map.pgm']}
    validate_reference_for_map(r,ROOT/'maps/map.yaml')
    r['map_sha256']['pgm']='0'*64
    with pytest.raises(ValueError,match='hashes'):validate_reference_for_map(r,ROOT/'maps/map.yaml')
    with pytest.raises(ValueError,match='scope'):require_reference_scope(5.,0.,r)
    assert before=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'maps').glob('map.*')}


def test_native_particle_cloud_reports_modes_without_inventing_filter_ess():
    cloud=ParticleCloud()
    for center in (-1.,1.):
        for i in range(30):
            p=Particle();p.pose.position.x=center+(i-15)*.003;p.weight=1.;cloud.particles.append(p)
    report=particle_spread(cloud)
    assert report['count']==60 and report['major_axis_density_modes']==2
    assert report['post_resample_effective_count']==pytest.approx(60)
    assert report['pre_resampling_effective_count_observable'] is False
    cloud.particles[0].weight=-1.
    with pytest.raises(ValueError,match='weights'):particle_spread(cloud)


def test_native_reference_accuracy_rejects_optimistic_covariance_and_stale_truth():
    from nav_msgs.msg import Odometry
    from turtlebot4.localization_health import reference_accuracy
    msg=Odometry();msg.header.frame_id='warehouse';msg.header.stamp.sec=10
    msg.pose.pose.orientation.w=1.
    assert reference_accuracy((.1,0.,0.),msg,10.,reference())['verified']
    assert not reference_accuracy((.26,0.,0.),msg,10.,reference())['verified']
    with pytest.raises(ValueError,match='synchronized'):
        reference_accuracy((0.,0.,0.),msg,11.,reference())
    msg.header.frame_id='map'
    with pytest.raises(ValueError,match='frame'):
        reference_accuracy((0.,0.,0.),msg,10.,reference())


def test_native_tf_delivery_defers_path_sample_until_grid_timestamp_is_available():
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer
    from turtlebot4.navigation_diagnostics import path_overlap_at_costmap_time
    buffer=Buffer()
    tf=TransformStamped();tf.header.frame_id='odom';tf.child_frame_id='map'
    tf.header.stamp.sec=10;tf.transform.rotation.w=1.
    buffer.set_transform(tf,'m4_3_test')
    grid=Costmap();grid.header.frame_id='odom';grid.header.stamp.sec=20
    grid.metadata.size_x=grid.metadata.size_y=4;grid.metadata.resolution=.5
    grid.metadata.origin.orientation.w=1.;grid.data=[0]*16
    path=RosPath();path.header.frame_id='map';path.poses=[PoseStamped()]
    assert path_overlap_at_costmap_time(buffer,path,grid) is None
    tf.header.stamp.sec=20;buffer.set_transform(tf,'m4_3_test')
    overlap=path_overlap_at_costmap_time(buffer,path,grid)
    assert overlap['inside']==1 and overlap['tf_stamp']==20.


def test_native_humble_byte_log_constants_and_message_levels_are_comparable():
    from rcl_interfaces.msg import Log
    from turtlebot4.navigation_diagnostics import log_severity
    message=Log();message.level=30
    assert log_severity(message.level)==log_severity(Log.WARN)==30
    assert log_severity(Log.ERROR)>log_severity(Log.WARN)
    json.dumps({'level':log_severity(message.level)})


def test_cancel_request_targets_known_owned_goal_id():
    import uuid
    from action_msgs.srv import CancelGoal
    from unique_identifier_msgs.msg import UUID
    goal_id=UUID(uuid=list(uuid.uuid4().bytes))
    request=CancelGoal.Request();request.goal_info.goal_id=goal_id
    assert request.goal_info.goal_id==goal_id and any(goal_id.uuid)


@pytest.mark.parametrize('count', [1, 3])
def test_navigation_acceptance_dispatches_exactly_one_destination(tmp_path, count):
    import importlib.util
    spec=importlib.util.spec_from_file_location('m4_3_verifier',ROOT/'scripts/verify_navigation.py')
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    path=tmp_path/'destination.json'
    path.write_text(json.dumps([{'name':'A-to-B','center':[-1.4,-.4]}]*count))
    if count==1:
        assert verifier.destination_centers(path,(0.,0.,0.))==[('A-to-B',(-1.4,-.4))]
        assert len(verifier.destination_centers(None,(0.,0.,0.)))==1
    else:
        with pytest.raises(ValueError,match='exactly one'):
            verifier.destination_centers(path,(0.,0.,0.))


def test_action_success_does_not_certify_physical_arrival_tolerance():
    import importlib.util
    spec=importlib.util.spec_from_file_location('m4_3_arrival',ROOT/'scripts/verify_navigation.py')
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    tolerances={'general_goal_checker.xy_goal_tolerance':.25,'general_goal_checker.yaw_goal_tolerance':.25}
    reference={'reference_uncertainty_m':.15,'reference_yaw_uncertainty_rad':.005}
    measured=verifier.physical_arrival_accuracy(.3116,.2499,tolerances,reference)
    assert measured['position_error_bound_m']==pytest.approx(.4616)
    assert not measured['within_configured_tolerance_verified']
    assert not verifier.physical_arrival_accuracy(.11,.02,tolerances,reference)['within_configured_tolerance_verified']
    assert verifier.physical_arrival_accuracy(.05,.02,tolerances,reference)['within_configured_tolerance_verified']
