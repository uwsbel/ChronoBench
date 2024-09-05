import pychrono

import pychrono.vehicle as vehicle

import rospy

from std_msgs.msg import String




chrono = pychrono.Chrono()




terrain = chrono.CreateTerrain("Terrain", chrono.ChBodyEasyInertia(1.0, 1.0, 1.0))

terrain.SetFriction(0.5)

terrain.SetRestitution(0.3)




hmmwv = vehicle.CreateHMMWV("HMMWV", chrono)

hmmwv.SetContactMethod(vehicle.ContactMethod.kDynamicFriction)

hmmwv.SetEngineType(vehicle.EngineType.kGasoline)

hmmwv.SetTireModel(vehicle.TireModel.kPacejka)




driver = vehicle.CreateDriver("Driver", chrono)

driver.SetControlMethod(vehicle.ControlMethod.kPID)




rospy.init_node('hmmwv_simulation')

pub_state = rospy.Publisher('vehicle_state', String, queue_size=10)




def clock_handler():

    chrono.SetTimeStep(0.01)


def driver_input_handler():

    

    pass


def vehicle_state_handler():

    

    state_message = "Vehicle state data"

    pub_state.publish(state_message)




chrono.RegisterClockHandler(clock_handler)

chrono.RegisterDriverInputHandler(driver_input_handler)

chrono.RegisterVehicleStateHandler(vehicle_state_handler)




while chrono.Ok():

    chrono.Advance()

    chrono.Synchronize()

    

    hmmwv.Update(chrono.Time())

    driver.Update(chrono.Time())

    terrain.Update(chrono.Time())


    

    rospy.spin()