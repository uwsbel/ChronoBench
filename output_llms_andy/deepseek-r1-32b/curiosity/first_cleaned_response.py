importchrono
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.ChSystem.SetChrono_VERBOSE(False)
system = chrono.ChSystemNSC()


ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().SetMaterialSurface(chrono.ChMaterialSurface())
ground.GetCollisionModel().AddBox(chrono.ChVectorD(10, 10, 1))
ground.GetVisualShape().SetTexture(chrono.ChTexture())
ground.GetVisualShape().SetTextureFilename("ground_texture.jpg")
system.Add(ground)


rover_body = chrono.ChBody()
rover_body.SetPos(chrono.ChVectorD(0, 0.5, 0))
rover_body.SetMass(200)
rover_body.GetCollisionModel().AddBox(chrono.ChVectorD(0.5, 0.5, 0.3))
rover_body.GetVisualShape().SetTexture(chrono.ChTexture())
rover_body.GetVisualShape().SetTextureFilename("rover_texture.jpg")
system.Add(rover_body)


wheel_pos = [(-0.4, 0.25, 0), (-0.4, -0.25, 0), (0.4, 0.25, 0), (0.4, -0.25, 0)]
for pos in wheel_pos:
    wheel = chrono.ChBody()
    wheel.SetPos(chrono.ChVectorD(*pos))
    wheel.SetMass(10)
    wheel.GetCollisionModel().AddCylinder(0.1, 0.2)
    wheel.GetVisualShape().SetTexture(chrono.ChTexture())
    wheel.GetVisualShape().SetTextureFilename("wheel_texture.jpg")
    system.Add(wheel)
    
    
    motor = chrono.ChWheelIECMotor()
    motor.SetMaxTorque(100)
    motor.SetSpeedTarget(10)
    wheel.AddDriver(motor)


driver = chrono.ChDriver()
rover_body.AddDriver(driver)


vis = chronoirr.ChIrrApp(system, "Curiosity Rover Simulation", chronoirr.dimension2du(800, 600))
vis.SetCamera(chronoirr.ChVectorf(5, 5, 5), chronoirr.ChVectorf(0, 0, 0))
vis.SetLight(chronoirr.ChVectorf(0, 1, 5), chronoirr.ChVectorf(0.8, 0.8, 0.8))
vis.EnableShadows()


logo = chronoirr.ChIrrLogo()
logo.SetPosition(chronoirr.ChVectorf(0.1, 0.1, 0))
vis.GetScene().Add(logo)


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    system.DoStepDynamics(0.001)
    
    
    key = vis.GetKeyboard()
    if key.KeyIsPressed(chronoirr.KEY_KEY_W):
        driver.GetSteering().SetSpeedTarget(10)
    elif key.KeyIsPressed(chronoirr.KEY_KEY_S):
        driver.GetSteering().SetSpeedTarget(-10)
    elif key.KeyIsPressed(chronoirr.KEY_KEY_A):
        driver.GetSteering().SetSteeringTarget(0.5)
    elif key.KeyIsPressed(chronoirr.KEY_KEY_D):
        driver.GetSteering().SetSteeringTarget(-0.5)
    else:
        driver.GetSteering().SetSpeedTarget(0)
        driver.GetSteering().SetSteeringTarget(0)