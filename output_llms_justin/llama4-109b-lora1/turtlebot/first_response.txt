import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# Initialize PyChrono environment and core components
ch = chrono.ChEngine("Turtlebot3")
ch.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
ch.SetSolverType(chrono.ChSolver.Type_BARZANEI)
ch.SetSolver(Iters=150)
ch.SetTimeStep(0.01)

# Create ground body
ground = chrono.ChBodyEasyBox(10, 1, 10, 1000)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ch.Add(ground)

# Initialize Turtlebot
turtlebot = veh.Turtlebot3("Turtlebot3")
turtlebot.Initialize()
turtlebot.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.0))
turtlebot.SetChaseCameraMultiStep(chrono.ChVector3d(0.0, 0.0, 0.0))
turtlebot.SetInitialLocation(chrono.ChVector3d(0, 0, 0.1))
turtlebot.SetInitialOrientation(chrono.ChQuaterniond(1, 0, 0, 0))
ch.Add(turtlebot.GetSystem())

# Create Irrlicht application
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(turtlebot.GetSystem())
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Turtlebot3')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.SetCameraMoveScale(5)
vis.SetLight(chrono.ChVector3d(0.5, 0.5, 0.5))

# Simulation loop
time = 0
left_motor = turtlebot.GetLeftMotor()
right_motor = turtlebot.GetRightMotor()
left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
left_motor.SetMaxSpeed(1.5)
right_motor.SetMaxSpeed(1.5)
left_motor.SetMaxTorque(10)
right_motor.SetMaxTorque(10)

while vis.Run() and not vis.WasUserQuit():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    ch.DoStepSimulation(0.01)
    time += 0.01
    if time > 5:
        left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_SINE)
        right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_SINE)
    if time > 10:
        left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
        right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
    if time > 15:
        left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_SAWTOOTH)
        right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_SAWTOOTH)
    if time > 20:
        left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_RAMP)
        right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_RAMP)
    if time > 25:
        left_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
        right_motor.SetMotorFunction(veh.ChTurtlebot3_MotorFunction_Type_CUBIC)
        left_motor.SetMaxSpeed(0.5)
        right_motor.SetMaxSpeed(0.5)
        left_motor.SetMaxTorque(5)
        right_motor.SetMaxTorque(5)
    if time > 30:
        vis.Exit()
vis.BindExitKeys()
vis.Close()