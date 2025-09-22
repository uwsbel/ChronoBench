import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemSMC()


vis = chronoirr.ChVisualSystemIrrlicht()


vis.SetWindowSize(1024, 768)


vis.SetWindowTitle("Epicyclic Gears Simulation")


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)


vis.SetCamera(chrono.ChVector3D(0, 0, 6), chrono.ChVector3D(0, 0, 0))


vis.AddTypicalLights()


truss = chrono.ChBodyEasyBox(chrono.ChVector3D(1, 1, 1), 1000, True, True, chrono.ChContactMaterialNSC())
truss.SetPos(chrono.ChVector3D(0, 0, 0))
sys.Add(truss)


bar = chrono.ChBodyEasyBox(chrono.ChVector3D(1, 1, 1), 1000, True, True, chrono.ChContactMaterialNSC())
bar.SetPos(chrono.ChVector3D(0, 0, 1))
sys.Add(bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
sys.Add(motor)


gear1 = chrono.ChBodyEasyCylinder(chrono.ChVector3D(0.5, 0.5, 0.5), 1000, True, True, chrono.ChContactMaterialNSC())
gear1.SetPos(chrono.ChVector3D(-1, 0, 0))
sys.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(chrono.ChVector3D(0.5, 0.5, 0.5), 1000, True, True, chrono.ChContactMaterialNSC())
gear2.SetPos(chrono.ChVector3D(1, 0, 0))
sys.Add(gear2)


gear_motor = chrono.ChLinkMotorRotationSpeed()
gear_motor.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVector3D(0, 0, 0), chrono.QUNIT))
gear_motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
sys.Add(gear_motor)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()