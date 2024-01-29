from simple_launch import SimpleLauncher
from nav2_common.launch import RewrittenYaml
import os

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('rviz', True)
base_path = os.path.abspath(os.path.dirname(__file__))

def launch_setup():

    sl.node('move','StampingCall')
    
    # run an EKF for wamv
    sl.node('robot_localization', 'ekf_node', name = 'ekf',
            parameters = [sl.find('move', 'ekf.yaml','params')],
            namespace = 'wamv',
            remappings = {'odometry/filtered': 'odom'})
                
    # run EKFs for friends NOT DONE YET
    
    # ekf_friend = sl.find('move', 'ekf_friend.yaml')
    # for friend in ('friend0', 'friend1'):

    #    configured_params = RewrittenYaml(source_file = ekf_friend,
    #                                      param_rewrites={'base_link_frame': friend},
    #                                      convert_types=True)

    #    sl.node('robot_localization', 'ekf_node', name = 'ekf',
    #            parameters = [configured_params],
    #            namespace = friend,
    #            remappings = {'odometry/filtered': 'odom'})
    
    if sl.arg('rviz'):
        sl.rviz(base_path + '/config.rviz')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)


