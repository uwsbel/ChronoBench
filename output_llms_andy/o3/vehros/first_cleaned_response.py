import math
import sys
import time


import pychrono as chrono
import pychrono.vehicle as veh
chrono.SetChronoDataPath(chrono.GetChronoDataPath())        
veh.SetDataPath(veh.GetDataPath())                          



import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Joy               


contact_method          = chrono.ChContactMethod_SMC
step_size               = 1.0e-3                 
step_size_tire          = 1.0e-3                 
render_step             = 1.0 / 50.0             
end_time                = 20.0                   


powertrain_type         = veh.PowertrainModelType_SHAFTS
tire_type               = veh.TireModelType_PAC89
driveline_type          = veh.DrivelineType_AWD


terrain_friction        = 0.8                    
terrain_restitution     = 0.1                    


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


init_loc   = chrono.ChVectorD(0, 0, 0.6)
init_rot   = chrono.ChQuaternionD(1, 0, 0, 0)    

vehicle    = veh.HMMWV_Vehicle(system,
                               fixed=False,
                               driveline_type=driveline_type)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.Initialize()


powertrain = veh.HMMWV_PowertrainShafts(powertrain_type)
vehicle.InitializePowertrain(powertrain)


tire_FL = veh.HMMWV_Tire(tire_type)
tire_FR = veh.HMMWV_Tire(tire_type)
tire_RL = veh.HMMWV_Tire(tire_type)
tire_RR = veh.HMMWV_Tire(tire_type)
vehicle.InitializeTire(tire_FL, veh.VehicleSide.LEFT, 0)
vehicle.InitializeTire(tire_FR, veh.VehicleSide.RIGHT, 0)
vehicle.InitializeTire(tire_RL, veh.VehicleSide.LEFT, 1)
vehicle.InitializeTire(tire_RR, veh.VehicleSide.RIGHT, 1)


terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(terrain_friction)
patch_mat.SetRestitution(terrain_restitution)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
    200.0, 200.0)                         
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)
patch.SetColor(chrono.ChColor(0.2, 0.5, 0.2))
terrain.Initialize()



class ROSDriver:
    def __init__(self):
        self.steer = 0.0     
        self.throttle = 0.0  
        self.brake = 0.0     
    def synchronize(self):
        vehicle.GetVehicle().GetVehicle().   
        pass
    def advance(self, dt):
        
        vehicle.GetVehicle().SetSteering(self.steer)
        vehicle.GetVehicle().SetThrottle(self.throttle)
        vehicle.GetVehicle().SetBraking(self.brake)

driver = ROSDriver()


class ChronoROSManager:
    def __init__(self, node_name="hmmwv_chrono"):
        rospy.init_node(node_name)
        
        self.pub_clock   = rospy.Publisher("/clock", Clock, queue_size=1)
        
        self.pub_state   = rospy.Publisher("/hmmwv/state", Twist, queue_size=1)
        
        self.sub_cmd     = rospy.Subscriber("/hmmwv/cmd", Joy, self.cmd_cb)
        
        self.driver      = driver

    
    def cmd_cb(self, msg):
        
        self.driver.steer    = max(-1.0, min(1.0,  msg.axes[0] if len(msg.axes)>0 else 0.0))
        self.driver.throttle = max(0.0,  min(1.0,  msg.axes[1] if len(msg.axes)>1 else 0.0))
        self.driver.brake    = max(0.0,  min(1.0,  msg.axes[2] if len(msg.axes)>2 else 0.0))

    
    def publish(self, time_chrono):
        
        clock_msg       = Clock()
        clock_msg.clock = rospy.Time.from_sec(time_chrono)
        self.pub_clock.publish(clock_msg)

        
        veh_body = vehicle.GetVehicle().GetChassisBody()
        pos      = veh_body.GetPos()
        vel      = veh_body.GetPos_dt()
        omg      = veh_body.GetWvel_loc()

        state_msg         = Twist()
        state_msg.linear.x = vel.x
        state_msg.linear.y = vel.y
        state_msg.linear.z = vel.z
        state_msg.angular.x = omg.x
        state_msg.angular.y = omg.y
        state_msg.angular.z = omg.z
        self.pub_state.publish(state_msg)

    
    def spinOnce(self):
        
        rospy.spin_once() if hasattr(rospy, "spin_once") else None   
        
        rospy.sleep(0.0)

ros_mgr = ChronoROSManager()


realtime_timer = chrono.ChRealtimeStepTimer()
chrono_time     = 0.0

print("==========  Starting simulation  ==========")
while (chrono_time < end_time) and (not rospy.is_shutdown()):
    
    ros_mgr.spinOnce()

    
    driver.synchronize()
    terrain.Synchronize()
    vehicle.Synchronize(chrono_time,
                        driver.throttle,
                        driver.steer,
                        driver.brake)

    
    driver.advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)

    
    ros_mgr.publish(chrono_time)

    
    realtime_timer.Spin(step_size)
    chrono_time += step_size

print("==========  Simulation finished  ==========")