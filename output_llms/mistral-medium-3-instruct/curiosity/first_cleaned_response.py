import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
ground.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


rover = chrono.ChBodyEasyBox(1.5, 0.5, 1.0, 500, True, True)
rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(rover)


wheel_radius = 0.25
wheel_width = 0.2
wheel_positions = [
    chrono.ChVectorD(0.8, 0.3, 0.5),  
    chrono.ChVectorD(0.8, 0.3, -0.5), 
    chrono.ChVectorD(0, 0.3, 0.5),    
    chrono.ChVectorD(0, 0.3, -0.5),   
    chrono.ChVectorD(-0.8, 0.3, 0.5), 
    chrono.ChVectorD(-0.8, 0.3, -0.5) 
]

wheels = []
for i, pos in enumerate(wheel_positions):
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 10, True, True)
    wheel.SetPos(pos)
    wheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.ChVectorD(0,0,1)))
    system.Add(wheel)
    wheels.append(wheel)

    
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, rover, chrono.ChCoordsysD(pos))
    system.Add(joint)


motor = chrono.ChLinkMotorRotationTorque()
motor.Initialize(wheels[0], rover, chrono.ChFrameD(wheel_positions[0]))
system.Add(motor)
motor.SetTorqueFunction(chrono.ChFunction_Const(5))  




for body in [ground, rover] + wheels:
    body.GetCollisionModel().SetFamily(1)
    body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)


application = chronoirr.ChIrrApp(system, "Curiosity Rover Simulation", chronoirr.dimension2du(1280, 720))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights(chrono.ChVectorD(30, 50, 30), chrono.ChVectorD(0, 0, 0))
application.AddTypicalCamera(chrono.ChVectorD(5, 2, 0))
application.AssetBindAll()
application.AssetUpdateAll()


camera = application.GetDevice().getSceneManager().addCameraSceneNode()
camera.setPosition(chrono.ChVectorD(5, 5, 5))
camera.setTarget(chrono.ChVectorD(0, 0, 0))


application.AddShadowAll()


application.SetTimestep(0.01)
while(application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    
    time = system.GetChTime()
    for i, wheel in enumerate(wheels):
        if i % 2 == 0:  
            wheel.SetWvel_loc(chrono.ChVectorD(0, 10*np.sin(time), 0))
        else:  
            wheel.SetWvel_loc(chrono.ChVectorD(0, -10*np.sin(time), 0))