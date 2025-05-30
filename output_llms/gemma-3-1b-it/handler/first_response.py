import time
import pychrono
import rospy
from pychrono.system import Chrono
from pychrono.system.system import System

# --- 1. Initialization & Core Components ---

# Define system parameters
system = System()
system.name = "BoxSimulation"
system.description = "A simple simulation of a box moving on a floor with gravity."
system.update_time_step = 1.0  # Update frequency - adjust for desired simulation speed
system.update_time_step_interval = 1.0 # Interval for updating time step
system.update_time_step_interval_interval = 1.0 # Interval for updating time step interval
system.gravity = 9.81  # Acceleration due to gravity (m/s^2)
system.material_density = 0.8 # Density of the box (kg/m^3) - adjust for realistic behavior
system.box_mass = 10.0 # Mass of the box (kg) - adjust for realistic behavior
system.box_size = 0.2 # Size of the box (m) - adjust for realistic behavior

# --- 2. Physical Systems & Objects ---

# Define the box
box = rospy.Action("box_move",
                    paramet=rospy.Parameter("x", type=float, default=0.0),
                    paramet=rospy.Parameter("y", type=float, default=0.0),
                    paramet=rospy.Parameter("z", type=float, default=0.0),
                    paramet=rospy.Parameter("mass", type=float, default=10.0),
                    paramet=rospy.Parameter("size", type=float, default=0.2),
                    reconnect=True)

# Define the floor
floor = rospy.Action("floor_move",
                    paramet=rospy.Parameter("x", type=float, default=0.0),
                    paramet=rospy.Parameter("y", type=float, default=0.0),
                    paramet=rospy.Parameter("z", type=float, default=0.0),
                    paramet=rospy.Parameter("mass", type=float, default=10.0),
                    paramet=rospy.Parameter("size", type=float, default=0.2),
                    reconnect=True)

# --- 3. ROS Handler ---

# Define the ROS handler
handler = rospy.Action("publish_message",
                         paramet=rospy.ActionParameters("message_topic", "box_movement_message"),
                         reconnect=True)

# --- 4. Simulation Loop ---

def run_simulation():
    rospy.init_node('box_simulation')
    system.subscribe(handler)
    system.wait_for_condition(system.gravity)  # Wait for gravity to be active

    try:
        while True:
            # Simulate movement
            system.update_time_step()
            system.move_box(x=system.box.x, y=system.box.y, z=system.box.z)
            system.update_time_step()
            # Simulate the floor moving
            floor.update_time_step()
            floor.move_box(x=floor.x, y=floor.y, z=floor.z)
            system.update_time_step()

            # Publish the message
            handler.publish(system.message)

            # Wait for a short period to control the simulation speed
            time.sleep(0.1)  # Adjust for desired simulation speed

    except rospy.EventRobotException:
        print("Robot exception occurred.  Stopping simulation.")
    except rospy.ROSInterruptException:
        print("Simulation interrupted.")
    finally:
        system.shutdown()


if __name__ == '__main__':
    run_simulation()