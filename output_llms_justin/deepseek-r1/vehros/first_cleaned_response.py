import os
import rospy
import pychrono as chrono
import pychrono.vehicle as veh
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray


rospy.init_node('chrono_simulation')
clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
odom_pub = rospy.Publisher('/hmmwv/odometry', Odometry, queue_size=10)


class DriverInputs:
    def __init__(self):
        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0

driver_inputs = DriverInputs()


def driver_inputs_callback(msg):
    if len(msg.data) >= 3:
        driver_inputs.steering = msg.data[0]
        driver_inputs.throttle = msg.data[1]
        driver_inputs.braking = msg.data[2]

rospy.Subscriber("/hmmwv/driver_inputs", Float64MultiArray, driver_inputs_callback)


chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
step_size = 0.001


init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()


time = 0.0
end_time = 100.0
while time < end_time and not rospy.is_shutdown():
    
    rospy.spin_once()
    
    
    cmd = veh.ChDriverInputs()
    cmd.m_steering = driver_inputs.steering
    cmd.m_throttle = driver_inputs.throttle
    cmd.m_braking = driver_inputs.braking
    
    
    hmmwv.Synchronize(time, cmd, terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    time += step_size
    
    
    clock_msg = Clock()
    clock_msg.clock = rospy.Time(time)
    clock_pub.publish(clock_msg)
    
    
    chassis = hmmwv.GetChassisBody()
    pos = chassis.GetPos()
    rot = chassis.GetRot()
    vel = chassis.GetPos_dt()
    ang_vel = chassis.GetWvel_loc()
    
    odom = Odometry()
    odom.header.stamp = rospy.Time(time)
    odom.header.frame_id = "world"
    odom.child_frame_id = "chassis"
    
    odom.pose.pose.position.x = pos.x
    odom.pose.pose.position.y = pos.y
    odom.pose.pose.position.z = pos.z
    odom.pose.pose.orientation.x = rot.e1
    odom.pose.pose.orientation.y = rot.e2
    odom.pose.pose.orientation.z = rot.e3
    odom.pose.pose.orientation.w = rot.e0
    
    vel_local = rot.RotateBack(vel)
    odom.twist.twist.linear.x = vel_local.x
    odom.twist.twist.linear.y = vel_local.y
    odom.twist.twist.linear.z = vel_local.z
    odom.twist.twist.angular.x = ang_vel.x
    odom.twist.twist.angular.y = ang_vel.y
    odom.twist.twist.angular.z = ang_vel.z
    
    odom_pub.publish(odom)