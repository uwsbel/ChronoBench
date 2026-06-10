import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# --- system: rovers use NSC + Bullet collision ---
system = chrono.ChSystemNSC()                                         # non-smooth system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # contact requires a collision system
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small rover contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small rover contact margin

# --- ground: fixed box, top surface at z = -0.5 under the spawn ---
ground_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 m ground box
ground.SetPos(chrono.ChVector3d(0, 0, -1))                           # top at z = -0.5
ground.SetFixed(True)                                                # ground does not move
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)                                                   # add ground to the system

# --- Viper rover: owns its bodies; DC-motor steering driver attached before Initialize ---
rover = robot.Viper(system)                                          # 6-wheel suspension rover, system-owned
driver = robot.ViperDCMotorControl()                                 # DC-motor control driver
rover.SetDriver(driver)                                              # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)                              # spawn position above the ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))               # Initialize takes a ChFramed

# --- sensor manager + scene lighting (camera-only) ---
manager = sens.ChSensorManager(system)                               # oversees all sensors
intensity = 1.0                                                      # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)  # point light for the camera

# --- third-person camera mounted on the rover chassis, looking forward/down ---
offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45),
                              chrono.QuatFromAngleAxis(.2, chrono.ChVector3d(0, 1, 0)))  # offset+tilt on chassis
cam = sens.ChCameraSensor(
    rover.GetChassis().GetBody(),                                    # attach to the rover chassis body
    15,                                                             # update rate (Hz) — physical rate
    offset_pose,                                                    # offset pose on the chassis
    720,                                                            # image width
    480,                                                            # image height
    1.408,                                                          # horizontal field of view (rad)
)
cam.SetName("Third Person POV")                                     # sensor name
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))  # live RGB preview window
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # SCORED CORE: save RGB PNG frames
manager.AddSensor(cam)                                              # push all filters BEFORE AddSensor

# --- Irrlicht visualization window (Initialize first, then scene elements; NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()                            # interactive review window
vis.AttachSystem(system)                                            # bind the system's visual assets
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                  # Z-up camera vertical (BEFORE Initialize)
vis.SetWindowSize(1280, 720)                                       # window size (BEFORE Initialize)
vis.SetWindowTitle("Viper rover - Rigid terrain")                 # window title (BEFORE Initialize)
vis.Initialize()                                                   # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo (AFTER Initialize)
vis.AddSkyBox()                                                    # sky box (AFTER Initialize)
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))  # window camera (AFTER Initialize)
vis.AddTypicalLights()                                             # standard lights (AFTER Initialize)
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # shadow-casting light

# --- control / timing parameters ---
time_step = 1e-3                                                    # integration step
max_steering = math.pi / 6                                         # practical max steering angle
sim_end = 14.0                                                     # simulation end time
render_step_size = 1.0 / 25                                        # render cadence: 25 FPS
render_steps = math.ceil(render_step_size / time_step)            # physics steps per rendered frame
step_number = 0                                                    # physics step counter


# --- main loop: ramp steering, update rover + sensors, step physics ---
time = 0
while vis.Run() and time < sim_end:
    time += time_step

    # ramp steering: hold straight, turn in, then back out
    steering = 0.0                                                 # default straight ahead
    if 2 < time < 7:
        steering = max_steering * (time - 2) / 5                  # ramp steering up
    elif 7 < time < 12:
        steering = max_steering * (12 - time) / 5                 # ramp steering down
    driver.SetSteering(steering)                                  # command the steering on the driver

    rover.Update()                                                # propagate command into the motors
    manager.Update()                                              # update ALL sensors once per step

    if step_number % render_steps == 0:
        vis.BeginScene()                                          # render the Irrlicht window
        vis.Render()
        vis.EndScene()
    step_number += 1                                              # advance the step counter

    system.DoStepDynamics(time_step)                              # advance the physics one step
