"""Viper rover on rigid terrain (PyChrono 9.0.x, Irrlicht).

Models the NASA-style Viper four-wheel rover driving on a flat rigid ground
patch. System type: ChSystemNSC (rigid contact between the rover wheels and the
ground). The main bodies are the Viper rover (chassis + 4 steerable, driven
wheels, created by the pychrono.robot.Viper wrapper) and a fixed ground body
carrying a contact material and a box collision/visual shape.

A ViperDCMotorControl driver spins all four wheels; the steering angle is ramped
smoothly from straight to a target over a steering window so the rover drives
forward and then curves. Expected behavior: the rover accelerates from rest,
translates across the terrain, and visibly turns as the steering ramps in.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as viper_robot


# === Constants === geometry, timing and control parameters (no bare literals downstream)
time_step = 1e-3            # NSC contact integration step (s)
sim_end = 12.0             # total simulated time (s)
render_fps = 50.0          # review render cadence (frames per simulated second)

ground_size_x = 30.0       # terrain extent along X (m)
ground_size_y = 30.0       # terrain extent along Y (m)
ground_thickness = 1.0     # terrain slab thickness (m)
ground_top_z = 0.0         # top surface height of the terrain (m)

rover_start_z = 0.0        # chassis spawn height above terrain top (m); wheels sit below
rover_init_pos = chrono.ChVector3d(0.0, 0.0, ground_top_z + rover_start_z)
rover_init_rot = chrono.QUNIT  # identity orientation, rover faces +X

ground_friction = 0.8      # tyre/soil friction coefficient
ground_restitution = 0.0   # inelastic ground contact

steer_start = 2.0          # steering ramp begins (s)
steer_end = 8.0            # steering ramp completes (s)
steer_target = 0.4         # final steering angle (rad)

motor_no_load_speed = math.pi  # drive-wheel free speed (rad/s)
motor_stall_torque = 300.0     # drive-wheel stall torque (N*m)

# === Precomputed derived values === computed ONCE before the loop
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
steer_span = steer_end - steer_start                          # precomputed once: ramp width (s)
ground_center_z = ground_top_z - 0.5 * ground_thickness       # precomputed once: slab center Z


# === System & gravity === ChSystemNSC with standard gravity along -Z
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
# Rover wheels contact the ground patch -> a narrow-phase collision system is required.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Ground === fixed rigid terrain patch with contact material + box collision
ground_mat = chrono.ChContactMaterialNSC()  # NSC material to match ChSystemNSC
ground_mat.SetFriction(ground_friction)
ground_mat.SetRestitution(ground_restitution)

ground = chrono.ChBodyEasyBox(
    ground_size_x, ground_size_y, ground_thickness,
    1000.0,       # density (unused while fixed, kept for completeness)
    True,         # visualization shape
    True,         # collision shape
    ground_mat,   # contact material
)
ground.SetPos(chrono.ChVector3d(0, 0, ground_center_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetColor(chrono.ChColor(0.55, 0.5, 0.45))
sys.Add(ground)

# === Rover & driver === Viper wrapper builds chassis + 4 steerable driven wheels
rover = viper_robot.Viper(sys)                 # chassis, arms, uprights, wheels + motors
driver = viper_robot.ViperDCMotorControl()     # DC-motor drive controller for the 4 wheels
driver.SetMotorNoLoadSpeed(motor_no_load_speed, viper_robot.V_LF)
driver.SetMotorNoLoadSpeed(motor_no_load_speed, viper_robot.V_RF)
driver.SetMotorNoLoadSpeed(motor_no_load_speed, viper_robot.V_LB)
driver.SetMotorNoLoadSpeed(motor_no_load_speed, viper_robot.V_RB)
driver.SetMotorStallTorque(motor_stall_torque, viper_robot.V_LF)
driver.SetMotorStallTorque(motor_stall_torque, viper_robot.V_RF)
driver.SetMotorStallTorque(motor_stall_torque, viper_robot.V_LB)
driver.SetMotorStallTorque(motor_stall_torque, viper_robot.V_RB)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(rover_init_pos, rover_init_rot))

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)   # Z-up world
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover on rigid terrain")
vis.Initialize()                                    # Initialize FIRST, then add scene nodes
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(4.0, -5.0, 3.0), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ground_top_z + 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))          # ground reference grid

# === Output setup === cache rover state getters; open review CSV
chassis_body = rover.GetChassis()  # cache: fetched once, reused every step


def steering_angle(t):
    """Smoothly ramp steering from 0 to steer_target across the steering window."""
    if t <= steer_start:
        return 0.0
    if t >= steer_end:
        return steer_target
    frac = (t - steer_start) / steer_span
    return steer_target * frac


# === Main loop === render once per frame; advance physics in an inner batch
frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            driver.SetSteering(steering_angle(t))
            rover.Update()
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state mid-run
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush CSV, build review video + timeseries plot
