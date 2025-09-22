import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


chrono.Init()


step_size = 2e-3
time_end = 100


init_loc = chrono.ChVectorD(0, 0, 1)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
vehicle = veh.HMMWV_FullVehicle(veh.ContactMethod_NSCM, veh.Irrlicht, False)


vehicle.SetContactMethod(chrono.ChContactMethod_NSCM)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
vehicle.SetTireType(veh.TireModelType_TMEASY)


vehicle.Initialize()


terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), chrono.ChVectorD(300, 300, 10))
patch.SetContactFrictionCoefficient(0.9)
patch.SetContactRestitutionCoefficient(0.01)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 400, 400)
terrain.Initialize()


driver = veh.ChIrrGuiDriver(vehicle, irr.createDevice(irr.video.E_DRIVER_TYPE.EDT_DIRECT3D9, irr.core.dim2du(800, 600)))


rospy.init_node('hmmwv_simulation', anonymous=True)


pub_clock = rospy.Publisher('/clock', Float64, queue_size=10)
pub_vehicle_state = rospy.Publisher('/vehicle_state', Twist, queue_size=10)

def callback_driver_inputs(data):
    
    driver.SetThrottle(data.linear.x)
    driver.SetSteering(data.angular.z)

sub_driver_inputs = rospy.Subscriber('/driver_inputs', Twist, callback_driver_inputs)


while vehicle.GetSystem().GetChTime() < time_end:
    time = vehicle.GetSystem().GetChTime()
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    driver.Synchronize(time)
    terrain.Synchronize(time)

    
    vehicle.Advance(step_size)
    driver.Advance(step_size)
    terrain.Advance(step_size)

    
    pub_clock.publish(Float64(time))
    vehicle_state = Twist()
    vehicle_state.linear.x = vehicle.GetPos().x
    vehicle_state.linear.y = vehicle.GetPos().y
    vehicle_state.linear.z = vehicle.GetPos().z
    pub_vehicle_state.publish(vehicle_state)

    
    rospy.rostime.wallsleep(0.01)


driver = None
vehicle = None
terrain = None
chrono.Shutdown()