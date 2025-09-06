import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.002)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


gear_material = chrono.ChContactMaterialSMC()
gear_material.SetFriction(0.1)       
gear_material.SetRestitution(0.0)     
gear_material.SetCompliance(1e-9)     




ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)


crank = chrono.ChBodyEasyBox(1.5, 0.1, 0.1, 1000, True, True)
crank.SetPos(chrono.ChVector3d(0, 0, 0))
crank.SetMass(5)
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.5, 0.5))
system.Add(crank)


sun_gear = chrono.ChBodyEasyCylinder(0.5, 0.2, 8000, True, True)
sun_gear.SetPos(chrono.ChVector3d(0, 0, 0))
sun_gear.SetFixed(True)
system.Add(sun_gear)


planet_gear = chrono.ChBodyEasyCylinder(0.3, 0.2, 6000, True, True)
planet_gear.SetPos(chrono.ChVector3d(1.0, 0, 0))
system.Add(planet_gear)




crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(ground, crank, 
                       chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), 
                                         chrono.QuatFromAngleY(math.pi/2)))
system.Add(crank_joint)


crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(ground, crank, 
                      chrono.ChFramed(chrono.ChVector3d(0, 0, 0), 
                                     chrono.QuatFromAngleY(math.pi/2)))
motor_func = chrono.ChFunction_Const(0.5)  
crank_motor.SetSpeedFunction(motor_func)
system.Add(crank_motor)


planet_joint = chrono.ChLinkLockRevolute()
planet_joint.Initialize(crank, planet_gear, 
                       chrono.ChCoordsysD(chrono.ChVector3d(1.0, 0, 0), 
                                         chrono.QuatFromAngleY(math.pi/2)))
system.Add(planet_joint)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Epicyclic Gear System')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(2, 1, 3))
vis.AddTypicalLights()
vis.AddSkyBox()


crank.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
sun_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
planet_gear.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))



time_step = 0.005
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

print("Simulation completed")