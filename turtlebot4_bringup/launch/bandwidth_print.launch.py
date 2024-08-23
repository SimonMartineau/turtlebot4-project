import os
import subprocess
import launch
from launch import LaunchDescription
from launch.actions import OpaqueFunction

def get_topics_and_bandwidth(context, *args, **kwargs):
    # Get the list of all topics
    topics = subprocess.getoutput("ros2 topic list").splitlines()

    # Iterate over each topic and get its bandwidth
    for topic in topics:
        print(f"Bandwidth for topic: {topic}")
        os.system(f"ros2 topic bw {topic}")

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=get_topics_and_bandwidth)
    ])
