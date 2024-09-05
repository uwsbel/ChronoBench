import chrono

from chrono_helpers import *

import irrlicht as ir




sch = chrono.ChSystemSMC()




ground = chrono.ChBoxShape(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(10, 1, 0.1))

ground_body = chrono.ChBodyE(mass=1000, shape=ground)

sch.AddBody(ground_body)




sch.SetGravity(chrono.ChVectorD(0, -9.81, 0))




viper_shape = chrono.ChBoxShape(chrono.ChVectorD(-0.5, -0.5, 0.5), chrono.ChVectorD(0.5, 0.5, 0.2))

viper_body = chrono.ChBodyE(mass=100, shape=viper_shape)

viper_body.SetPos(chrono.ChVectorD(0, 0, 0.5))

sch.AddBody(viper_body)




driver_shape = chrono.ChBoxShape(chrono.ChVectorD(-0.1, -0.1, 0.2), chrono.ChVectorD(0.1, 0.1, 0.1))

driver_body = chrono.ChBodyE(mass=5, shape=driver_shape)

driver_body.SetPos(chrono.ChVectorD(0, 0, 0.5))

sch.AddBody(driver_body)




scene = chrono.ChVisualSceneIrrlicht()

scene.AddGround(ground_body)




camera = chrono.ChArbCameraIrrlicht()

camera.SetCameraVerticalFOV(90)

camera.SetCameraPosition(chrono.ChVectorD(0, 5, 10))

camera.SetFocalPoint(chrono.ChVectorD(0, 0, 0))




light = chrono.ChLightIrrlicht()

light.SetLightColor(chrono.ChColor(1, 1, 1))

light.SetLightPosition(chrono.ChVectorD(0, 10, 10))




scene.AddCamera(camera)

scene.AddLight(light)




def simulation_loop(sch, viper_body, driver_body, scene, camera, light, duration=10):

    time = 0

    while time < duration:

        sch.DoStepDynamics(1/60)

        viper_body.SetSteering(chrono.ChVectorD(0, 0.01 * time, 0))

        driver_body.SetSteering(chrono.ChVectorD(0, 0.01 * time, 0))


        

        scene.BeginScene()

        scene.DrawAll()

        scene.EndScene()


        time += 1/60




simulation_loop(sch, viper_body, driver_body, scene, camera, light)