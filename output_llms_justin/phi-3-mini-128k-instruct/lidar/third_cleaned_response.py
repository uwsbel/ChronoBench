import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math



def initialize_vehicle_and_driver():
    vehicle = chrono.ChArticulatedBodyEasy(chrono.ChVector3d(0, 0, 0), chrono.ChQuaternion(1, 0, 0, 0))
    vehicle.SetName("ARTcar")
    vehicle.SetMass(1000)
    vehicle.SetInertiaXX(1000, 1000, 1000)
    vehicle.SetLocalInertia([1, 1, 1])
    vehicle.SetChassis(chrono.ChBodyEasyBox(1, 1, 1, 1000))
    vehicle.SetCollisionModel(chrono.ChCollisionShape_Box(chrono.ChVector3d(0, 0, 0), 10))
    vehicle.SetMotor(chrono.ChWheelMotor(1, 1, 1, 0, 0))
    vehicle.SetDriver(chrono.ChArticulatedBodyDriver())

    driver = chrono.ChArticulatedBodyDriver(vehicle)
    return vehicle, driver