import time
import numpy as np
from pychrono import Chrono, System
from pychrono.utils import get_time_unit
import rospy
from std_msgs.msg import Float64

# Define system parameters
gravity = 9.81  # m/s^2
floor_mass = 1000  # kg
box_mass = 200  # kg
box_size = 1.0  # m
box_position = [0, 0]  # m
box_velocity = [0, 0]  # m/s
simulation_duration = 10  # seconds
timestep = 0.01  # seconds

# Create the Chrono system
system = Chrono(
    system_name="MySimulation",
    gravity=gravity,
    material_properties={"density": 2700, "elastic_modulus": 2.1e9},  # Example material properties
    body_type="box",
    body_position=[box_position],
    body_velocity=[box_velocity],
    simulation_duration=simulation_duration,
    # You can add more system parameters here if needed
)

# Define the physical system
# This is a simplified example.  A more sophisticated system would include
# collision detection, friction, etc.
# This example assumes a simple rigid body interaction.
# Consider using a more robust physics engine for realistic simulations.

# Create a fixed floor
floor = System()
floor.add_object(system)
floor.set_position([0, 0, 0])

# Create a movable box
box = System()
box.add_object(system)
box.set_position([box_position[0], box_position[1], box_position[2]])
box.set_velocity([0, 0])


# Create a ROS message handler
def publish_message(message):
    rospy.loginfo("Publishing message: %s", message)
    rospy.message_path("MySimulation_Update") # Assign a unique message path

# ROS Manager Setup
rospy.init_node('MySimulation')
rospy.loginfo("ROS Manager initialized.")

# Main Simulation Loop
while True:
    # Simulate the system
    system.update()

    # Publish the update message
    publish_message(system.update())

    # Delay for simulation duration
    time.sleep(timestep)