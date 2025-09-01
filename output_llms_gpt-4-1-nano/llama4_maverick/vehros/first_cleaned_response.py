import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import rospy
from std_msgs.msg import Float64


chrono.Init()


sys = chrono.ChSystemNSC()


init_file = "hmmwv/vehicle/HMMWV_Vehicle.json"
vehicle = veh.HMMWV_Vehicle(sys, init_file=init_file)





vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.Q_from_AngZ(0)))


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)
terrain.Initialize()


driver = veh.ChDriver(vehicle.GetVehicle())


driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0
driver_inputs.m_throttle = 0
driver_inputs.m_braking = 0


rospy.init_node('hmmwv_simulator', anonymous=True)


pub_vehicle_state = rospy.Publisher('/vehicle_state', Float64, queue_size=10)

def clock_sync_callback(msg):
    
    pass

def driver_inputs_callback(msg):
    
    driver_inputs.m_steering = msg.data  
    driver.SetInputs(driver_inputs)

rospy.Subscriber("/clock", Float64, clock_sync_callback)
rospy.Subscriber("/driver_inputs", Float64, driver_inputs_callback)  


step_size = 2e-3
time_end = 100


realtime_timer = chrono.ChRealtimeStepTimer()
while sys.GetChTime() < time_end:
    time = sys.GetChTime()
    driver_inputs.m_throttle = 0.5  
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    sys.DoStepDynamics(step_size)
    
    
    pub_vehicle_state.publish(Float64(vehicle.GetSpeed()))  
    
    
    rospy.rostime.wallsleep(0.01)
    
    
    realtime_timer.Spin(step_size)