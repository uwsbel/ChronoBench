import chrono as ch
import chrono_vehicle as chv
import chrono_ros as chr
import rospy


ch.ChronoEnvironment.set_G_acc(9.81)


simulation = ch.ChronoSimulation()


vehicle = chv.HMMWV(simulation.GetSystem())


vehicle.SetContactMethod(ch.ContactMethod_NC)
vehicle.SetEngineType(ch.EngineType_SHAFTS)
vehicle.SetTireModel(ch.TireModel_Pacejka)


terrain = chv.GroundPlane(simulation.GetSystem())
terrain.SetFrictionCoefficient(0.8)
terrain.SetRestitutionCoefficient(0.7)


driver = chv.DriverInputs(simulation.GetSystem())


ros_manager = chr.ROSManager(simulation)


def clock_cb(time):
    ros_manager.Update(time)

def driver_inputs_cb(inputs):
    driver.SetInputs(inputs)

def vehicle_state_cb(state):
    ros_manager.SetVehicleState(state)

ros_manager.RegisterClockCallback(clock_cb)
ros_manager.RegisterDriverInputsCallback(driver_inputs_cb)
ros_manager.RegisterVehicleStateCallback(vehicle_state_cb)


def simulate():
    rate = rospy.Rate(100)  

    while not rospy.is_shutdown():
        
        simulation.Advance(0.01)  

        
        ros_manager.Update(simulation.GetChronoTime())

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('hmmwv_simulation')

    simulate()