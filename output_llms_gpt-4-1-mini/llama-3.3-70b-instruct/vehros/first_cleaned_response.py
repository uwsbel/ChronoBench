import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.mkl as mkl
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystem()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 100))
vis.AddTypicalCameraMotionControls()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")


vehicle = veh.HMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetEngineType(chrono.ChEngineType_INTERNAL_COMBUSTION)
vehicle.SetTireModel(chrono.ChTireModel_SIMPLE)


terrain = chrono.ChBodyEasyBox(sys, 100, 100, 1, 1000, True, chrono.ChVectorD(0, -10, 0))
terrain.SetFriction(0.8)
terrain.SetRestitution(0.2)


sys.Add(vehicle)
sys.Add(terrain)


driver = veh.ChDriver()
driver.SetVehicle(vehicle)
driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


rospy.init_node('hmmwv_simulation')
clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
driver_input_pub = rospy.Publisher('/driver_input', Twist, queue_size=10)
vehicle_state_pub = rospy.Publisher('/vehicle_state', JointState, queue_size=10)


def clock_callback(msg):
    sys.SetChTime(msg.clock.secs + msg.clock.nsecs * 1e-9)

def driver_input_callback(msg):
    driver.SetSteeringInput(msg.angular.z)
    driver.SetThrottleInput(msg.linear.x)
    driver.SetBrakingInput(msg.linear.y)

rospy.Subscriber('/clock', Clock, clock_callback)
rospy.Subscriber('/driver_input', Twist, driver_input_callback)


while True:
    
    sys.DoStepDynamics(0.01)

    
    driver.Synchronize(0.01)

    
    clock_msg = Clock()
    clock_msg.clock.secs = int(sys.GetChTime())
    clock_msg.clock.nsecs = int((sys.GetChTime() % 1) * 1e9)
    clock_pub.publish(clock_msg)

    driver_input_msg = Twist()
    driver_input_msg.angular.z = driver.GetSteeringInput()
    driver_input_msg.linear.x = driver.GetThrottleInput()
    driver_input_msg.linear.y = driver.GetBrakingInput()
    driver_input_pub.publish(driver_input_msg)

    vehicle_state_msg = JointState()
    vehicle_state_msg.position = [vehicle.GetChassis().GetPos().x(), vehicle.GetChassis().GetPos().y(), vehicle.GetChassis().GetPos().z()]
    vehicle_state_msg.velocity = [vehicle.GetChassis().GetPos_dt().x(), vehicle.GetChassis().GetPos_dt().y(), vehicle.GetChassis().GetPos_dt().z()]
    vehicle_state_pub.publish(vehicle_state_msg)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    if vis.GetExit():
        break