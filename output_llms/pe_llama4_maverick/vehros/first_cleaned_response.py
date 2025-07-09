import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import rospy
from rospkg import RosPack
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np


contact_method = chrono.ChContactMethod_NSC
engine_type = veh.HMMWV_EngineType_SIMPLE
tire_model = veh.HMMWV_TireType_RIGID
terrain_length = 100.0  
terrain_width = 100.0  
terrain_friction = 0.9
terrain_restitution = 0.0
init_loc = chrono.ChVector3d(0, 0.5, -5)  
init_rot = chrono.Q_from_AngZ(0)  





vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetEngineType(engine_type)
vehicle.SetTireType(tire_type=tire_model)
vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), terrain_length, terrain_width)
terrain.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())


rospy.init_node('hmmwv_simulation', anonymous=True)


class ROSManager:
    def __init__(self):
        self.pub_pose = rospy.Publisher('/hmmwv/pose', PoseStamped, queue_size=10)
        self.pub_inputs = rospy.Publisher('/hmmwv/inputs', Float64MultiArray, queue_size=10)

    def publish_pose(self, pose):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = pose.pos.x
        msg.pose.position.y = pose.pos.y
        msg.pose.position.z = pose.pos.z
        msg.pose.orientation.x = pose.rot.e0
        msg.pose.orientation.y = pose.rot.e1
        msg.pose.orientation.z = pose.rot.e2
        msg.pose.orientation.w = pose.rot.e3
        self.pub_pose.publish(msg)

    def publish_inputs(self, steering, throttle, braking):
        msg = Float64MultiArray()
        msg.data = [steering, throttle, braking]
        self.pub_inputs.publish(msg)

ros_manager = ROSManager()


def clock_cb(msg):
    
    pass

rospy.Subscriber('/clock', rospy.AnyMsg, clock_cb)


def driver_inputs_cb(msg):
    steering = msg.data[0]
    throttle = msg.data[1]
    braking = msg.data[2]
    driver.SetSteering(steering)
    driver.SetThrottle(throttle)
    driver.SetBraking(braking)

rospy.Subscriber('/hmmwv/driver_inputs', Float64MultiArray, driver_inputs_cb)


step_size = 2e-3


while not rospy.is_shutdown():
    time = vehicle.GetSystem().GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Update(time, driver_inputs)
    terrain.Update(time)
    
    
    chassis_pose = vehicle.GetChassis().GetPose()
    ros_manager.publish_pose(chassis_pose)
    ros_manager.publish_inputs(driver_inputs.Steering(), driver_inputs.Throttle(), driver_inputs.Braking())
    
    
    vehicle.GetSystem().DoStepDynamics(step_size)
    
    
    rospy.sleep(step_size)