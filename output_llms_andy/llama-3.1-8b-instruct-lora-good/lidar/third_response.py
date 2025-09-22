import pychrono as chrono
import py.sensor as sens
import numpy as np
import time
import math
import pychrono.vehicle as veh

def main():
    ----------------
    # Create the system
    ----------------
    mphysical = veh.ChSystem()
    mph.SetCollisionSystemType(veh.Collision_type_BULLET)
    mph.SetGravitationalAcceleration(chrono.ChVector3(0, 0, -9.81, 0))
    mph.SetSolver(chrono.ChSolver.Type_SORCASAM)

    #--------------------------------
    Add vehicle
    --------------------------------
    # Create vehicle and initialize
    vehicle = veh.RTMEtireCar()
    vehicle.SetContactMethod(chrono.ChContactMethod_Scircular)
    vehicle.SetChassis(chrono.Chassis(1.4, 1.6, 2.1, 1.1, 1,1.2,1.2,1, 1.2, 0.2, 0.2, 0.2, 0.2,0.2 0, 0,0.2 0.0, 0,0 0.0,0, 0.0,0,0 0.0,0 0.0,0, 0.0, 0,0.0,0,0,0, 0,0,0.0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0 0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
print("error happened with only start ```python")