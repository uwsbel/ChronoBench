import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')  
sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.5)
material.SetDampingF(0.1)
material.SetCompliance(0.01)


truss = chrono.ChBody()
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetMass(0)
truss.SetFixed(True)  
truss.EnableCollision(True)
sys.Add(truss)


bar_length = 4.0
bar = chrono.ChBodyEasyBox(bar_length, 0.1, 0.1, 1000, True, True, material)
bar.SetPos(chrono.ChVector3d(0, 0, 0))
bar.SetRot(chrono.Q_from_AngZ(math.pi / 2))  
sys.Add(bar)


gear1_radius = 0.5
gear2_radius = 1.0


gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.1, 1000, True, True, material)
gear1.SetPos(chrono.ChVector3d(0, 0, 0))
gear1.SetRot(chrono.Q_from_AngZ(0))  
sys.Add(gear1)


gear2 = chrono.ChBodyEasyCylinder(gear2_radius, 0.1, 1000, True, True, material)
gear2.SetPos(chrono.ChVector3d(0, 0, gear1_radius + gear2_radius))  
gear2.SetRot(chrono.Q_from_AngZ(math.pi))  
sys.Add(gear2)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngZ(0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
sys.Add(motor)


truss_shape = chrono.ChVisualShapeBox(4, 0.1, 0.1)
truss.AddVisualShape(truss_shape)

bar_shape = chrono.ChVisualShapeBox(bar_length, 0.1, 0.1)
bar.AddVisualShape(bar_shape)

gear1_shape = chrono.ChVisualShapeCylinder(gear1_radius, 0.1)
gear1.AddVisualShape(gear1_shape)

gear2_shape = chrono.ChVisualShapeCylinder(gear2_radius, 0.1)
gear2.AddVisualShape(gear2_shape)


while vis.Run():
    sys.DoStepDynamics(0.01)  
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

print("Simulation completed.")