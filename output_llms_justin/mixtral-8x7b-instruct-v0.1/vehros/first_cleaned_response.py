import chrono
from chrono import ChVector, ChCoordsys, ChFrame
from chrono.collision import ChCollisionModel
from chrono.vehicle import ChVehicleModel, ChTireModelList, ChTire, ChEngine, ChLinkTSDA, ChAssystemHMMWV
from chrono.sensor import ChVisualization, ChColor
from chrono.fea import ChElementTetra
from chrono.irrlicht import ChirrApp
from pychrono.vehicle import ChDriver, ChControllerGenVel, ChControllerSteer, ChControllerBrake
from pychrono.sensor import ChSensorTireForce
from pychrono.system import ChSystemSMC
from pychrono.irrlicht import ChIrrAppSMC
import time
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


my_system = ChSystemSMC()



terrain = my_system.AddTerrain(0, 0, 100, 100, 0.1)
terrain.SetContactMethod(chrono.ChContactMethod.SMC)
terrain.SetFriction(0.5)
terrain.SetRestitution(0.1)


vehicle = ChAssystemHMMWV(my_system)
vehicle.SetSuspensionStiffness(200000)
vehicle.SetSuspensionDamping(15000)
vehicle.SetTireStiffness(1e6)
vehicle.SetTireDamping(5000)
vehicle.SetTireFriction(1.2)
vehicle.SetTireRestitution(0.2)


driver = ChDriver()
vehicle.SetDriver(driver)


def clock_synchronization_handler(current_time):
    my_system.SetTime(current_time.to_sec())

def driver_inputs_handler(data):
    driver.SetSteerAngle(data.position[0])
    driver.SetGasPedal(data.position[1])
    driver.SetBrakePedal(data.position[2])

def vehicle_state_handler(data):
    pass  

rospy.init_node('pychrono_hmmwv_simulation')
rospy.Subscriber('/driver_inputs', JointState, driver_inputs_handler)
vehicle_state_publisher = rospy.Publisher('/vehicle_state', JointState, queue_size=10)
rospy.Timer(rospy.Duration(0.01), clock_synchronization_handler)


while not rospy.is_shutdown():
    
    driver_inputs = ChControllerGenVel()
    driver_inputs.AddController(ChControllerSteer(0.1))
    driver_inputs.AddController(ChControllerBrake(0.0))
    driver.SetController(driver_inputs)

    
    my_system.DoStepDynamics(1 / 60.0)

    
    vehicle_state = JointState()
    vehicle_state.name = ['steer', 'gas', 'brake']
    vehicle_state.position = [driver.GetSteerAngle(), driver.GetGasPedal(), driver.GetBrakePedal()]
    vehicle_state_publisher.publish(vehicle_state)

    time.sleep(1 / 60.0)


my_app = ChIrrAppSMC(my_system)
my_app.AddTypicalSky()
my_app.AddTypicalLogo()
my_app.AddCamera(chrono.ChCoordsys(0, 10, 5))
my_app.Add(vehicle.GetChassis().GetChBody().GetChVisualization())
my_app.Add(ChVisualization(terrain, ChColor(0.5, 0.5, 0.5)))
my_app.AddSensor(ChVisualization(vehicle, ChColor(0.5, 0.5, 1)))
my_app.AddSensor(ChVisualization(vehicle.GetTire(0), ChColor(1, 0, 0)))
my_app.AddSensor(ChVisualization(vehicle.GetTire(1), ChColor(0, 1, 0)))
my_app.AddSensor(ChVisualization(vehicle.GetTire(2), ChColor(0, 0, 1)))
my_app.AddSensor(ChVisualization(vehicle.GetTire(3), ChColor(1, 1, 0)))
my_app.AddSensor(ChSensorTireForce(vehicle.GetTire(0)))
my_app.AddSensor(ChSensorTireForce(vehicle.GetTire(1)))
my_app.AddSensor(ChSensorTireForce(vehicle.GetTire(2)))
my_app.AddSensor(ChSensorTireForce(vehicle.GetTire(3)))
my_app.AddLight(chrono.ChVector(10, 10, -10), 0.5, 0.5, 0.5)
my_app.AddLight(chrono.ChVector(-10, 10, -10), 0.5, 0.5, 0.5)

my_app.SetTimestep(1 / 60.0)
my_app.SetTryRealtime(True)
my_app.SetWindowSize(1280, 720)
my_app.Initialize()
my_app.Run()