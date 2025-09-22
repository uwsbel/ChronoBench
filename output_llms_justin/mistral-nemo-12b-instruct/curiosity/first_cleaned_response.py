import pychrono as chrono
from pychrono.irrlicht import *


simulation = chrono.ChSimulation()
irrlicht = IrrlichtApplication(simulation, "Curiosity Rover Simulation")
simulation.SetTimestepper(chrono.ChTimestepper.SolidNSC())
simulation.SetCollisionSystemType(chrono.ChCollisionSystem.Neptune)
simulation.SetCollisionConfig(chrono.ChCollisionConfig.CreateSolidCollisionConfig())
simulation.SetSolverType(chrono.ChSolver.PSSOR())
simulation.SetSolverMaxIterations(50)
simulation.SetSolverTolerance(1e-3)



ground = chrono.ChBodyEasyBox(100, 100, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -5))
ground.SetBodyFixed(True)
ground.GetMaterial().SetFriction(0.9)
simulation.Add(ground)


rover = chrono.ChBodyEasyCylinder(0.5, 1, 1000, True, True)
rover.SetPos(chrono.ChVectorD(0, 0, 5))
rover.SetBodyFixed(False)
rover.GetMaterial().SetFriction(0.9)
simulation.Add(rover)


motor = chrono.ChLinkMotor()
motor.Initialize(rover, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetPos(0)
motor.SetMaxForce(1000)
simulation.Add(motor)



rover.SetPos(chrono.ChVectorD(0, 0, 5))


rover.SetPos_dt(chrono.ChVectorD(0, 0, 0))


force = chrono.ChVectorD(0, 0, -100)
rover.ApplyForce(force, chrono.ChVectorD(0, 0, 5))


irrlicht.SetCamera(chrono.ChVectorD(0, -10, 10), chrono.ChVectorD(0, 0, 0))
irrlicht.SetLightType(LightType.Directional)
irrlicht.SetLightDirection(chrono.ChVectorD(1, 1, -1))
irrlicht.SetLightColor(chrono.ChColor(1, 1, 1))
irrlicht.EnableShadows(True)
irrlicht.SetShadowMapSize(1024)
irrlicht.SetShadowMapResolution(1024)


ground.GetVisualShape(0).SetTexture("ground_texture.png")
rover.GetVisualShape(0).SetTexture("rover_texture.png")
irrlicht.AddLogo("logo.png", 0.1, 0.1)


while not irrlicht.IsDone():
    simulation.DoStepDynamics(0.01)
    irrlicht.DoStep()