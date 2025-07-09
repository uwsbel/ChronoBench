import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
import math
import time

# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.robot as rob
import pychrono.irrlicht as chronoirr

# Initialize ROS node
rospy.init_node('hmmwv_simulation', anonymous=True)

# Publisher for vehicle state
state_pub = rospy.Publisher('vehicle_state', PoseStamped, queue_size=10)

# Subscriber for driver inputs
steering_sub = rospy.Subscriber('steering', Float32, lambda msg: driver.set_steering(msg.data))
throttle_sub = rospy.Subscriber('throttle', Float32, lambda msg: driver.set_throttle(msg.data))
brake_sub = rospy.Subscriber('brake', Float32, lambda msg: driver.set_brake(msg.data))

# Initialize PyChrono system
sys = chrono.ChSystemSMC()
sys.SetContactMethod(chrono.ChContactMethod.BULLET)

# Create HMMWV vehicle
hmmwv = veh.HMMWV(sys, True, True)
hmmwv.SetContactMethod(sys.GetContactMethod())
hmmwv.SetEngineType(veh.EngineType.ELECTRIC)
hmmwv.SetTireType(veh.TireModelType.PACEJK)
hmmwv.Initialize()

# Set initial position and orientation
init_pos = chrono.ChVectorD(0, 0, 1)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))

# Create terrain
terrain = veh.RigidTerrain(sys)
terrain_material = chrono.ChMaterialSurface()
terrain_material.SetFriction(0.8)
terrain_material.SetRestitution(0.2)
terrain.AddPatch(terrain_material, chrono.ChCoordsysD(), 100, 100)
terrain.Initialize()

# Initialize driver
class ROSDriver:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0

    def set_steering(self, value):
        self.steering = value

    def set_throttle(self, value):
        self.throttle = value

    def set_brake(self, value):
        self.brake = value

    def update(self):
        self.vehicle.GetDriver().SetThrottle(self.throttle)
        self.vehicle.GetDriver().SetSteering(self.steering)
        self.vehicle.GetDriver().SetBrake(self.brake)

driver = ROSDriver(hmmwv)

# Simulation loop
time_step = 0.01
last_ros_time = rospy.Time.now().to_sec()

while not rospy.is_shutdown():
    # Synchronize ROS time
    current_ros_time = rospy.Time.now().to_sec()
    if current_ros_time - last_ros_time >= time_step:
        # Update vehicle and driver
        driver.update()
        sys.DoStepDynamics(time_step)

        # Publish vehicle state
        state = PoseStamped()
        state.header.stamp = rospy.Time.now()
        pos = hmmwv.GetChassis().GetPos()
        state.pose.position.x = pos.x
        state.pose.position.y = pos.y
        state.pose.position.z = pos.z
        rot = hmmwv.GetChassis().GetRot()
        state.pose.orientation.w = rot.w
        state.pose.orientation.x = rot.x
        state.pose.orientation.y = rot.y
        state.pose.orientation.z = rot.z
        state_pub.publish(state)

        last_ros_time = current_ros_time

# Cleanup
rospy.signal_shutdown('Simulation completed')