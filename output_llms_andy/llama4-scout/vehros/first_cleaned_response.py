import chrono as ch
import chrono_vehicle as chv
import chrono_ros as chr
import rospy


ch.InitializeSystem()


system = ch.ChronoSystemMultibody()


vehicle_file = "path/to/hmmwv/vehicle_hmmwv.json"
vehicle = chv.ChronoVehicle(system, vehicle_file)


vehicle.SetContactMethod(ch.ContactMethod.NSC)
vehicle.SetEngineType(ch.EngineType.Simple)
vehicle.SetTireModel(ch.TireModel.Rigid)


terrain_file = "path/to/terrain/terrain_irregular.json"
terrain = chv.ChronoTerrain(system, terrain_file)
terrain.SetFrictionCoefficient(0.8)
terrain.SetRestitutionCoefficient(0.7)


driver = chv.ChronoDriver(vehicle)
driver.SetSteeringType(chv.SteeringType.PID)
driver.SetThrottleType(chv.ThrottleType.PID)
driver.SetBrakingType(chv.BrakingType.PID)


chr.ChronoRosManager(system)


def clock_cb(time):
    system.Update()

def driver_inputs_cb(inputs):
    driver.SetInputs(inputs)

def vehicle_state_cb(state):
    vehicle.SetState(state)


rospy.init_node('hmmwv_simulation')


chr_sub = rospy.Subscriber('chrono_clock', ch.ChronoClock, clock_cb)
driver_inputs_pub = rospy.Publisher('driver_inputs', ch.ChronoDriverInputs, driver_inputs_cb)
vehicle_state_pub = rospy.Publisher('vehicle_state', ch.ChronoVehicleState, vehicle_state_cb)


rate = rospy.Rate(100.0)  
while not rospy.is_shutdown():
    
    system.Advance(0.01)  
    
    
    chr.ChronoRosManager.Update()
    
    rate.sleep()