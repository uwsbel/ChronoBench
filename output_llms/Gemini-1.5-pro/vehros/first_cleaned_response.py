import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry






timestep = 1e-3
terrain_length = 100.0
terrain_width = 100.0
gravity = chrono.ChVectorD(0, -9.81, 0)


vehicle_model = 'HMMWV'  
engine_type = veh.VehicleModel.Engine.ENGINE_TYPE_SparkIgnition  
tire_model = veh.PacejkaTire  
contact_method = chrono.ChContactMethod_NSC  


terrain_mu = 0.8  
terrain_cr = 0.2  


clock_topic = '/clock'
driver_input_topic = '/vehicle/driver_input'
vehicle_state_topic = '/vehicle/state'






chrono.SetChronoDataPath(veh.GetDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(gravity)


rospy.init_node('chrono_simulator', anonymous=True)






vehicle = veh.HMMWV(system)
vehicle.SetChassisFixed(False)
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0)))
vehicle.SetEngineType(engine_type)
vehicle.SetTireType(tire_model)
vehicle.SetTireCollisionType(veh.TireCollisionType_TRIANGLE_MESH)






terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(), 
                         chrono.ChVectorD(-terrain_length / 2, 0, -terrain_width / 2), 
                         chrono.ChVectorD(terrain_length / 2, 0, terrain_width / 2), 
                         0.1)
patch.SetContactFrictionCoefficient(terrain_mu)
patch.SetContactRestitutionCoefficient(terrain_cr)
terrain.Initialize()






class SimpleDriver:
    def __init__(self):
        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0

    def update(self):
        
        pass

driver = SimpleDriver()






def clock_callback(msg):
    global timestep
    timestep = msg.data

def driver_input_callback(msg):
    
    pass

rospy.Subscriber(clock_topic, Float64, clock_callback)
rospy.Subscriber(driver_input_topic, Odometry, driver_input_callback)
vehicle_state_pub = rospy.Publisher(vehicle_state_topic, Odometry, queue_size=10)





while not rospy.is_shutdown():
    
    driver.update()

    
    vehicle.SetSteering(driver.steering)
    vehicle.SetThrottle(driver.throttle)
    vehicle.SetBraking(driver.braking)

    
    system.DoStepSize(timestep)

    
    
    msg = Odometry()
    
    vehicle_state_pub.publish(msg)

    
    rospy.sleep(timestep)