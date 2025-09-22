import pychrono as chrono
import pychrono.irrlicht as irr
import math

# Initialize PyChrono environment and core components
core = chrono.ChCore()
core.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
core.SetSolverType(chrono.ChSolver.Type_PENDANT)
core.SetSolver(Iters=50)
core.SetTimeStep(0.01)

# Create the Irrlicht application
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(core.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gears Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.0, 1.5))
vis.SetAmbientLight(chrono.ChColor(0.2, 0.2, 0.2))
vis.EnableDirectionalLight(chrono.ChVector3d(0.0, 0.0, -1.0), chrono.ChColor(1.0, 1.0, 1.0))

# Create the fixed truss
truss = chrono.ChBody()
truss.SetFixed(True)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
core.GetSystem().Add(truss)

# Create the rotating bar
bar = chrono.ChBody()
bar.SetPos(chrono.ChVector3d(0.5, 0, 0))
bar.SetMass(1)
bar.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
core.GetSystem().Add(bar)

# Create the gears
gear1 = chrono.ChBody()
gear1.SetPos(chrono.ChVector3d(0, 0, 0))
gear1.SetMass(1)
gear1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
core.GetSystem().Add(gear1)

gear2 = chrono.ChBody()
gear2.SetPos(chrono.ChVector3d(0.25, 0, 0))
gear2.SetMass(1)
gear2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
core.GetSystem().Add(gear2)

# Create the gear motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, chrono.ChFrame(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)), gear2, chrono.ChFrame(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
core.GetSystem().AddLinkMotor(motor)

# Create the contacts and collisions
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.3)
contact_mat.SetRestitution(0.5)

# Create the Irrlicht visualization shapes
bar_shape = irr.ChVisualShapeIrrlicht()
bar_shape.SetShapeType(irr.VisualShapeType_BOX)
bar_shape.SetColor(chrono.ChColor(1.0, 0.0, 0.0))
bar_shape.SetDimensions(chrono.ChVector3d(0.1, 0.1, 1.0))
bar.AddVisualShape(bar_shape)

gear1_shape = irr.ChVisualShapeIrrlicht()
gear1_shape.SetShapeType(irr.VisualShapeType_CYLINDER)
gear1_shape.SetColor(chrono.ChColor(0.0, 1.0, 0.0))
gear1_shape.SetDimensions(chrono.ChVector3d(0.1, 0.1, 0.5))
gear1.AddVisualShape(gear1_shape)

gear2_shape = irr.ChVisualShapeIrrlicht()
gear2_shape.SetShapeType(irr.VisualShapeType_CYLINDER)
gear2_shape.SetColor(chrono.ChColor(0.0, 0.0, 1.0))
gear2_shape.SetDimensions(chrono.ChVector3d(0.1, 0.1, 0.5))
gear2.AddVisualShape(gear2_shape)

# Run the simulation
while vis.Run() :
    core.DoStepSimulation(vis.GetRenderMode())
    vis.Render()
    vis.UpdateCamera()