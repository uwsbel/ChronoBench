import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle


my_system = chrono.ChSystemNSC()


my_system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


material_nsc = chrono.ChContactMaterialNSC()
material_nsc.SetFriction(0.5)
material_nsc.SetDampingF(0.1)
material_nsc.SetCompliance(0.01)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Simulation Window')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


body_car = chrono.ChBody()
body_car.SetPos(chrono.ChVector3d(0, 0, 0))
body_car_shape = chrono.ChVisualShapeBox(1.0, 1.0, 1.0)
body_car.AddVisualShape(body_car_shape)
my_system.Add(body_car)


link_wheel = chrono.ChLinkLockRevolute()
link_wheel.Initialize(body_car, body_ground, chrono.ChFramed())
my_system.Add(link_wheel)


while vis.Run():
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


driver = robot.RS_Driver(body_car, body_ground, True)
driver.SetDriver(True)


tire_model = vehicle.TMEasyTireModel(body_car, body_ground, material_nsc)
tire_model.SetFriction(0.5)
tire_model.SetDampingF(0.1)
tire_model.SetCompliance(0.01)


driver.SetInteractive(True)


while True:
    
    
    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()