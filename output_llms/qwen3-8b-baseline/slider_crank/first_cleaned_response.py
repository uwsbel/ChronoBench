import pychrono as chrono
from pychrono import irrlicht
from pychrono.irrlicht import *
import numpy as np


system = chrono.ChSystemNSC()


floor = chrono.ChBodyEasyBox(10, 10, 0.1, 1000, True, True)
floor.SetPos(chrono.ChVector(0, 0, -0.05))
floor.SetFixed(True)
system.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(chrono.ChVector(0, 0, 0), 0.1, 1.0, 1000, True, True)
crankshaft.SetPos(chrono.ChVector(0, 0, 0))
crankshaft.SetFixed(False)
system.Add(crankshaft)


joint_crank_floor = chrono.ChLinkRevolute()
joint_crank_floor.Initialize(crankshaft, floor, chrono.ChVector(0, 0, 0), chrono.ChVector(0, 0, 0), chrono.ChVector(0, 0, 1))
system.AddLink(joint_crank_floor)


motor = chrono.ChLinkMotorSpeed()
motor.Initialize(crankshaft, floor, chrono.ChVector(0, 0, 0), chrono.ChVector(0, 0, 1))
motor.SetAngularSpeed(chrono.ChVector(10, 0, 0))  
system.AddLink(motor)


connecting_rod = chrono.ChBodyEasyCylinder(chrono.ChVector(0, 0, 0), 0.05, 2.0, 1000, True, True)
connecting_rod.SetPos(chrono.ChVector(0.5, 0, 0))  
connecting_rod.SetFixed(False)
system.Add(connecting_rod)


joint_crank_rod = chrono.ChLinkRevolute()
joint_crank_rod.Initialize(crankshaft, connecting_rod, chrono.ChVector(0.5, 0, 0), chrono.ChVector(0.5, 0, 0), chrono.ChVector(0, 0, 1))
system.AddLink(joint_crank_rod)


piston = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
piston.SetPos(chrono.ChVector(2.5, 0, 0))  
piston.SetFixed(False)
system.Add(piston)


joint_rod_piston = chrono.ChLinkSliderPin()
joint_rod_piston.Initialize(connecting_rod, piston, chrono.ChVector(2.5, 0, 0), chrono.ChVector(2.5, 0, 0), chrono.ChVector(1, 0, 0))
system.AddLink(joint_rod_piston)


application = irrlicht.ChIrrApp(system, "Crank-Slider Mechanism", irrlicht.dimension2d_i(800, 600))
application.AddCamera(chrono.ChCamera())
application.SetCameraPosition(chrono.ChVector(5, 2, 3))
application.SetCameraTarget(chrono.ChVector(0, 0, 0))
application.SetCameraVerticalAlignment(chrono.ChCamera.VerticalAlignment.TOP)


application.SetLightModel(chrono.ChLightModel.ambient)
application.SetAmbientLight(chrono.ChColor(0.5, 0.5, 0.5))
application.SetDirectionalLight(chrono.ChVector(0, -1, -1), chrono.ChColor(0.8, 0.8, 0.8), chrono.ChVector(0, 0, 0))



texture = chrono.ChTexture("path_to_texture.png")
piston.GetVisualShape().AddTexture(texture)
logo = chrono.ChLogo()
application.AddLogo(logo)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(0.01)