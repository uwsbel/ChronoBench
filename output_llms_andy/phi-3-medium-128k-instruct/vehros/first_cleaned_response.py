import pychrono

import pychrono.vehicle as vehicle

import pychrono.robotics as robotics

import pychrono.chrono as chrono

import pychrono.synchronizer as synchronizer

import rospy

from std_msgs.msg import String




chrono.Init()




engine_type = chrono.EngineType.kGasoline

contact_method = chrono.ContactMethod.kNewton

tire_model = chrono.RubberTire




hmmwv = vehicle.Vehicle(chrono.VehicleType.kHMMWV)

hmmwv.SetContactMethod(contact_method)

hmmwv.SetEngineType(engine_type)

hmmwv.SetTireModel(tire_model)




terrain_friction = 0.5

terrain_restitution = 0.2




terrain = chrono.ChBody()

terrain.SetFriction(terrain_friction)

terrain.SetRestitution(terrain_restitution)

terrain.SetMass(10000)

terrain.SetInertiaXX(chrono.ChBoxInertia(10000, 10000, 10000))

terrain.SetPos(chrono.ChVector(0, 0, 0))

terrain.SetRot(chrono.ChQuaternion(1, 0, 0, 0))




driver = robotics.Driver()




rospy.init_node('hmmwv_simulation')

pub_state = rospy.Publisher('vehicle_state', String, queue_size=10)




while not rospy.is_shutdown():

    

    hmmwv.Advance(0.01)

    terrain.Advance(0.01)

    driver.Update(0.01)


    

    synchronizer.Sync(hmmwv, driver, terrain)

    vehicle_state = hmmwv.GetVehicleState()

    pub_state.publish(str(vehicle_state))


    

    rospy.sleep(0.01)