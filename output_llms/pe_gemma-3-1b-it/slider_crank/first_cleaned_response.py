import chrono
import numpy as np
import math
import os
import sys

def create_crank_slider_simulation():
    

    
    try:
        chrono = chrono.ChSystem()
        chrono.SetChronoDataPath('/path/to/data')  
    except Exception as e:
        print(f"Error initializing Chrono: {e}")
        return

    
    
    floor = chrono.ChBody()
    floor.SetPos(chrono.ChVector3d(0, 0, 0))
    floor.SetMass(100)
    floor.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
    floor.SetFixed(True)
    floor.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  
    floor.SetContactMethod(chrono.ChContactMethod.Type_SIMPLE)

    
    crank = chrono.ChLinkLockRevolute()
    crank.SetAsRectangularSection(chrono.ChVector3d(1, 0, 0), chrono.ChVector3d(0, 1, 0))
    crank.SetYoungModulus(0.01e9)
    crank.SetShearModulus(0.01e9 * 0.3)
    crank.SetRayleighDamping(0.000)
    crank.SetPos(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0)))
    crank.SetSection(floor)
    crank.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    crank.SetContactMethod(chrono.ChContactMethod.Type_SIMPLE)


    
    connecting_rod = chrono.ChLinkLockRevolute()
    connecting_rod.SetAsRectangularSection(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
    connecting_rod.SetYoungModulus(0.01e9)
    connecting_rod.SetShearModulus(0.01e9 * 0.3)
    connecting_rod.SetRayleighDamping(0.000)
    connecting_rod.SetPos(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    connecting_rod.SetSection(floor)
    connecting_rod.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    connecting_rod.SetContactMethod(chrono.ChContactMethod.Type_SIMPLE)


    
    piston = chrono.ChLinkLockRevolute()
    piston.SetAsRectangularSection(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
    piston.SetYoungModulus(0.01e9)
    piston.SetShearModulus(0.01e9 * 0.3)
    piston.SetRayleighDamping(0.000)
    piston.SetPos(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    piston.SetSection(floor)
    piston.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    piston.SetContactMethod(chrono.ChContactMethod.Type_SIMPLE)

    
    crank.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    piston.SetSpeedFunction(chrono.ChFunction_Const(1.0))
    connecting_rod.SetSpeedFunction(chrono.ChFunction_Const(1.0))

    
    motor = chrono.ChLinkLockLock()
    motor.Initialize(crank, connecting_rod, chrono.ChFramed())
    motor.SetMotionY(chrono.ChVector3d(0, 0, 0))
    motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))

    
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Crank-Slider Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 

    
    vis.Run()

    print("Simulation completed.")

if __name__ == "__main__":
    create_crank_slider_simulation()