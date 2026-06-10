import math
import pychrono as chrono
import pychrono.irrlicht as irr


import pychrono.robot as robot






step_size = 1.0e-3
render_step_size = 1.0 / 60.0

terrain_length = 30.0
terrain_width = 30.0
terrain_thickness = 0.20
terrain_friction = 0.9

init_loc = chrono.ChVector3d(0.0, 0.0, 0.45)
init_rot = chrono.QUNIT

forward_speed = 0.35          
steering_start = 0.0
steering_end = 0.6            
steering_ramp_time = 6.0      

sim_end_time = 20.0






def clamp(x, xmin, xmax):
    return max(xmin, min(x, xmax))


def set_driver_command(driver, speed, steering):
    
    if driver is None:
        return

    
    if hasattr(driver, "SetDesiredVelocity"):
        driver.SetDesiredVelocity(speed, steering)
        return

    
    if hasattr(driver, "SetDesiredSpeed"):
        driver.SetDesiredSpeed(speed)
    elif hasattr(driver, "SetSpeed"):
        driver.SetSpeed(speed)
    elif hasattr(driver, "SetThrottle"):
        driver.SetThrottle(speed)

    
    if hasattr(driver, "SetDesiredSteering"):
        driver.SetDesiredSteering(steering)
    elif hasattr(driver, "SetSteering"):
        driver.SetSteering(steering)
    elif hasattr(driver, "SetSteeringInput"):
        driver.SetSteeringInput(steering)






system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))


system.SetNumThreads(1)
system.GetSolver().SetMaxIterations(80)






ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(terrain_friction)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(
    terrain_length,
    terrain_width,
    terrain_thickness,
    1000.0,          
    True,            
    True,            
    ground_mat
)
ground.SetName("rigid_terrain")
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, -terrain_thickness / 2.0))


if ground.GetVisualShape(0):
    ground.GetVisualShape(0).SetColor(chrono.ChColor(0.45, 0.45, 0.45))

system.Add(ground)






rover = robot.Viper(system)


if hasattr(rover, "SetChassisVisualizationType"):
    try:
        rover.SetChassisVisualizationType(chrono.VisualizationType_MESH)
    except Exception:
        pass

if hasattr(rover, "SetWheelVisualizationType"):
    try:
        rover.SetWheelVisualizationType(chrono.VisualizationType_MESH)
    except Exception:
        pass


driver = None
if hasattr(robot, "ViperDCMotorControl"):
    try:
        driver = robot.ViperDCMotorControl(rover)
    except TypeError:
        driver = robot.ViperDCMotorControl()
elif hasattr(robot, "ViperSpeedDriver"):
    try:
        driver = robot.ViperSpeedDriver(rover)
    except TypeError:
        driver = robot.ViperSpeedDriver()

if driver is not None and hasattr(rover, "SetDriver"):
    rover.SetDriver(driver)


init_frame = chrono.ChFramed(init_loc, init_rot)

try:
    rover.Initialize(init_frame)
except TypeError:
    
    if hasattr(rover, "SetInitPosition"):
        rover.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    rover.Initialize()


set_driver_command(driver, forward_speed, steering_start)






vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("PyChrono VIPER Rover on Rigid Terrain")
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()


vis.AddCamera(
    chrono.ChVector3d(4.0, -6.0, 3.0),
    chrono.ChVector3d(0.0, 0.0, 0.5)
)






realtime_timer = chrono.ChRealtimeStepTimer()

render_steps = max(1, int(math.ceil(render_step_size / step_size)))
step_number = 0

while vis.Run():
    time = system.GetChTime()

    if time >= sim_end_time:
        break

    
    ramp = clamp(time / steering_ramp_time, 0.0, 1.0)
    steering = steering_start + ramp * (steering_end - steering_start)

    
    set_driver_command(driver, forward_speed, steering)

    
    if hasattr(rover, "Update"):
        rover.Update()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    system.DoStepDynamics(step_size)

    
    realtime_timer.Spin(step_size)

    step_number += 1