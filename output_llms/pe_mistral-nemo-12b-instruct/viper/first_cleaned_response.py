import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_PATH'], 'vehicle'))


sys = chrono.ChSystemSMC()
sys.Set_Gravity(chrono.ChVector3d(0, -9.81, 0))


ground_mat = chrono.ChMaterialSurfaceNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetDampingF(0.2)
ground_body = chrono.ChBody()
ground_body.SetMaterial(ground_mat)
ground_body.SetBodyFixed(True)
ground_body.SetPos(chrono.ChVector3d(0, -0.5, 0))
ground_body_shape = chrono.ChVisualShapeBox(chrono.ChVector3d(20, 0.1, 20))
ground_body.AddVisualShape(ground_body_shape)
sys.Add(ground_body)


rover = veh.Viper()
rover.SetChassisCollisionType(veh.ChViper.ChassisCollisionType_BOX)
rover.SetContactMethod(veh.ChViper.ContactMethod_HF)
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
rover.Initialize()


driver = veh.ViperDriver()
driver.SetSteeringAngle(0)
driver.SetThrottle(0)
driver.SetBrake(0)
rover.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Viper Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5))
vis.AddTypicalLights()


steering_angle = 0
steering_rate = 0.1
sim_time = 0
while vis.Run():
    
    steering_angle += steering_rate
    if steering_angle >= 0.5:
        steering_rate = -0.1
    elif steering_angle <= -0.5:
        steering_rate = 0.1
    driver.SetSteeringAngle(steering_angle)

    
    sys.DoStepDynamics(0.01)
    sim_time += 0.01

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()