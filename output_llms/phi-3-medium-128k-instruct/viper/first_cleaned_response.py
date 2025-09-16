import chrono

import chrono_irrlicht_sensor as irr

import chrono_irrlicht_vis as chronovis

import numpy as np

import math




chrono.init()




ground_body = chrono.ChBodyAux()

ground_body.SetName("Ground")

ground_body.SetMass(0)

ground_body.SetDynamic(False)

ground_body.SetInertiaTensor(chrono.ChVector3(0, 0, 0))

ground_body.SetExtents(chrono.ChVector3(100, 100, 0.1))

ground_body.SetPos(chrono.ChVector3(0, 0, 0))

ground_body.SetCoM(chrono.ChVector3(0, 0, 0))




ground_body_module = chrono.ChBodyEasyRigidBodyModule(ground_body)




viper_rover = chrono.ChBodyAux()

viper_rover.SetName("Viper Rover")

viper_rover.SetMass(500)

viper_rover.SetDynamic(True)

viper_rover.SetInertiaTensor(chrono.ChVector3(1000, 1000, 1000))

viper_rover.SetCoM(chrono.ChVector3(0, 0, 0))




viper_rover_module = chrono.ChBodyEasyRigidBodyModule(viper_rover)




driver = chrono.ChDriverUNB(viper_rover)

driver.SetName("Driver")

driver.SetUseZeroStep(False)




steering_time = 10.0

steering_angle = 0.0

steering_rate = math.pi / steering_time




vis = chronovis.ChIrrApp()

vis.SetUseZBuffer(1)

vis.SetUseGUI(1)

vis.SetUseFog(0)

vis.SetUseLighting(1)

vis.SetUsePerspectiveProjection(1)

vis.SetCamera(chrono.ChVector3(0, 0, 10), chrono.ChVector3(0, 0, 0), chrono.ChVector3(0, 0, -1), 30, 90)




vis.AddBody(ground_body)

vis.AddBody(viper_rover)




module = chrono.ChSystemNSC()

module.Add(ground_body_module)

module.Add(viper_rover_module)

module.Add(driver)




while chrono.GetChTime() < steering_time:

    

    steering_angle += steering_rate

    driver.Update(chrono.ChTime(0.01))

    

    module.DoStepDynamics(chrono.ChTime(0.01))

    

    vis.BeginScene()

    vis.DrawModels()

    vis.EndScene()




chrono.reset()