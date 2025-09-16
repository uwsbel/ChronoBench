import chrono as ch
import chrono_vehicle as chv
import chrono_ros as chr
import rospy


ch.Initialize()


system = ch.ChronoSystemMultibody()


vehicle_file = "path/to/hmmwv/vehicle_hmmwv.json"
vehicle = chv.ChronoVehicle(system, vehicle_file)


vehicle.SetContactMethod(ch.ContactMethod_NSC)
vehicle.SetEngineType(ch.EngineType_SHAFTS)
vehicle.SetTireModel(ch.TireModel_FM)


terrain_file = "path/to/terrain/terrain_irregular.json"
terrain = chv.ChronoTerrain(system, terrain_file)
terrain.SetFrictionCoefficient(0.8)
terrain.SetRestitutionCoefficient(0.7)


driver = chv.ChronoDriver(vehicle)
driver.SetSteeringType(chv.SteeringType_PURE_PURSUIT)


chr.ChronoRosManager(system)


def clock_cb(time):
    system.Update()

def driver_inputs_cb(inputs):
    driver.SetInputs(inputs)

def vehicle_state_cb(state):
    vehicle.SetState(state)


chr.RegisterClockCallback(clock_cb)
chr.RegisterDriverInputsCallback(driver_inputs_cb)
chr.RegisterVehicleStateCallback(vehicle_state_cb)


def simulate():
    rate = rospy.Rate(100)  
    while not rospy.is_shutdown():
        
        system.Advance(0.01)  
        driver.Advance(0.01)
        terrain.Advance(0.01)
        
        
        chr.UpdateRosManager()
        
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("hmmwv_simulation")
    simulate()