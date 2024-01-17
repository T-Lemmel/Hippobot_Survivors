from simple_launch import SimpleLauncher
import os

sl = SimpleLauncher(use_sim_time=True)
base_path = os.path.abspath(os.path.dirname(__file__))

def launch_setup():

    sl.node('mapping', 'cloud_reader')
    sl.node('mapping', 'obstacle_node.py')
    sl.node('mapping', 'map_node.py')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
