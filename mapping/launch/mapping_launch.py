from simple_launch import SimpleLauncher
import os

sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('rviz', False)
base_path = os.path.abspath(os.path.dirname(__file__))

def launch_setup():

    sl.node('mapping', 'DetectAndPublishCentroids')
    sl.node('mapping', 'FillMap')

    if sl.arg('rviz'):
        sl.rviz(base_path + '/config.rviz')
    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
