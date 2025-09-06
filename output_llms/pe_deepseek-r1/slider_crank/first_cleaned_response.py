import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


material = chrono.ChContactMaterialNSC()
material.SetFriction(0.2)
material.SetDampingF(0.01)


ground = chrono.ChBody()
ground.SetPos(chrono.ChVector3d(0, 0, 0))
ground.SetFixed(True)
system.Add(ground)


ground_shape = chrono.ChVisualShapeBox(2, 0.1, 2)
ground_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
ground.AddVisualShape(ground_shape)




crank_length = 0.3
rod_length = 0.7
crank_width = 0.05
piston_size = chrono.ChVector3d(0.1, 0.1, 0.1)


crank = chrono.ChBody()
crank.SetPos(chrono.ChVector3d(0, 0.5, 0))
crank.SetMass(5)
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))
system.Add(crank)


crank_shape = chrono.ChVisualShapeCylinder(crank_width, crank_length)
crank_shape.SetColor(chrono.ChColor(0.8, 0.0, 0.0))
crank.AddVisualShape(crank_shape, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), 
                 chrono.QuatFromAngleY(chrono.CH_PI_2))


rod = chrono.ChBody()
rod.SetPos(chrono.ChVector3d(crank_length/2 + rod_length/2, 0.5, 0))
rod.SetMass(3)
rod.SetInertiaXX(chrono.ChVector3d(0.05, 0.5, 0.05))
system.Add(rod)


rod_shape = chrono.ChVisualShapeBox(rod_length, crank_width, crank_width)
rod_shape.SetColor(chrono.ChColor(0.0, 0.8, 0.0))
rod.AddVisualShape(rod_shape)


piston = chrono.ChBody()
piston.SetPos(chrono.ChVector3d(crank_length + rod_length, 0.5, 0))
piston.SetMass(2)
piston.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
system.Add(piston)


piston_shape = chrono.ChVisualShapeBox(piston_size.x, piston_size.y, piston_size.z)
piston_shape.SetColor(chrono.ChColor(0.0, 0.0, 0.8))
piston.AddVisualShape(piston_shape)




crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(ground, crank, 
                      chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0), 
                                  chrono.QuatFromAngleZ(0)))
system.Add(crank_joint)


rod_joint1 = chrono.ChLinkLockRevolute()
rod_joint1.Initialize(crank, rod, 
                     chrono.ChFramed(chrono.ChVector3d(crank_length, 0.5, 0), 
                                 chrono.QuatFromAngleZ(0)))
system.Add(rod_joint1)


rod_joint2 = chrono.ChLinkLockRevolute()
rod_joint2.Initialize(rod, piston, 
                     chrono.ChFramed(chrono.ChVector3d(crank_length + rod_length, 0.5, 0), 
                                 chrono.QuatFromAngleZ(0)))
system.Add(rod_joint2)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(ground, piston, 
                       chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0), 
                                   chrono.QuatFromAngleY(chrono.CH_PI_2)))
system.Add(piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(ground, crank, 
                chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0), 
                            chrono.QuatFromAngleZ(0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_PI))  
system.Add(motor)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1.5, 0.5, 1.5))
vis.AddTypicalLights()



time_step = 0.005
simulation_time = 0
max_simulation_time = 10

while vis.Run() and simulation_time < max_simulation_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    simulation_time += time_step

print("Simulation completed successfully!")

motor.SetSpeedFunction(chrono.ChFunction_Const(2 * chrono.CH_PI))  

piston_shape.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))

piston.EnableCollision(True)
   piston.GetCollisionModel().AddShape(chrono.ChCollisionShapeBox(piston_size.x, piston_size.y, piston_size.z))

if simulation_time > 5:
       vis.GetActiveCamera().setPosition(chronoirr.vector3df(0, 1, 2))