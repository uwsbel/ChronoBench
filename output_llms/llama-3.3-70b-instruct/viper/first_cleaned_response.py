import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


ground_body = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, False)
ground_body.SetPos(chrono.ChVectorD(0, -1, 0))
ground_body.SetBodyFixed(True)


viper_rover = chrono.ChBodyEasyBox(system, 1, 0.5, 1, 10, False, True)
viper_rover.SetPos(chrono.ChVectorD(0, 1, 0))


driver = chrono.ChDriver(system, viper_rover)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCameraGimbal(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))


vis.GetCamera().SetFov(45)
vis.GetCamera().SetFocalLength(1000)


vis.GetLight(0).SetDiffuseColor(chrono.ChVectorF(1, 1, 1))
vis.GetLight(0).SetSpecularColor(chrono.ChVectorF(1, 1, 1))


dt = 0.01  
t_end = 10  
t = 0  


while t < t_end:
    
    steering_angle = np.sin(t)
    driver.SetSteeringAngle(steering_angle)

    
    system.DoStepDynamics(dt)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    t += dt

    
    print(f"Time: {t:.2f}")


del vis