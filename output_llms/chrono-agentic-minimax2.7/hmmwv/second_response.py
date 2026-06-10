"""
HMMWV following a circular path with PID steering controller.

Wheeled vehicle (mbs_in_scene): HMMWV_Full on rigid terrain,
following a circular path with constant throttle and PID steering.

System: ChSystemNSC with NSC contact materials.
"""

import math, os, csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Simulation parameters ===
time_step = 1e-3
sim_end = 30.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Terrain dimensions (increased from 100 to 200)
terrain_length = 200.0
terrain_width = 200.0

# Circular path parameters
path_radius = 20.0   # reasonable radius for circular path
path_run_in = 10.0   # straight run-in before circular arc
target_speed = 12.0  # m/s

# Throttle constant (scripted, per prompt requirement)
CONSTANT_THROTTLE = 0.3

# === Chrono data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create HMMWV ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
# Start at origin, chassis at default height
init_loc = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.QUNIT
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Create rigid terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    terrain_length,
    terrain_width,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Create circular path ===
# Start well above ground so chassis clears terrain
start = chrono.ChVector3d(-path_run_in, 0, 0.5)
path = veh.CirclePath(start, path_radius, path_run_in, True, 3)  # 3 laps, left turn

# === Path-follower driver with PID steering ===
driver = veh.ChPathFollowerDriver(
    hmmwv.GetVehicle(),
    path,
    "circle_path",
    target_speed,
)
# PID gains for steering: KP=0.8, KI=0, KD=0 (per driver skill PID layout)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)
# Speed controller gains
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Path visualization: sentinel and target spheres ===
# Sentinel marker
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_sphere = chrono.ChVisualShapeSphere(0.15)
sentinel_sphere.SetColor(chrono.ChColor(0.0, 1.0, 0.0))  # green
sentinel_marker.AddVisualShape(sentinel_sphere, chrono.ChFramed())
system.AddBody(sentinel_marker)

# Target marker
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_sphere = chrono.ChVisualShapeSphere(0.15)
target_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))  # red
target_marker.AddVisualShape(target_sphere, chrono.ChFramed())
system.AddBody(target_marker)

# === Irrlicht visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()  # vehicle demo: directional light
vis.AddTypicalLights()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Visualization of path points via two small balls (as static assets) ===
# The sentinel/target spheres above update each frame; add two static marker balls
# for the path balls mentioned in the prompt (visual reference at path center)
path_ball1 = chrono.ChBody()
path_ball1.SetFixed(True)
ball1_shape = chrono.ChVisualShapeSphere(0.5)
ball1_shape.SetColor(chrono.ChColor(1.0, 1.0, 0.0))  # yellow
path_ball1.AddVisualShape(ball1_shape, chrono.ChFramed())
path_ball1.SetPos(chrono.ChVector3d(start.x, start.y, 0.25))
system.AddBody(path_ball1)

# Second path visualization ball at 90 degrees around the circle
path_ball2 = chrono.ChBody()
path_ball2.SetFixed(True)
ball2_shape = chrono.ChVisualShapeSphere(0.5)
ball2_shape.SetColor(chrono.ChColor(0.0, 1.0, 1.0))  # cyan
path_ball2.AddVisualShape(ball2_shape, chrono.ChFramed())
path_ball2.SetPos(chrono.ChVector3d(start.x + path_radius, start.y, 0.25))
system.AddBody(path_ball2)

# === Precompute cache ===
steer_ctrl = driver.GetSteeringController()  # cache: steering controller

# === Review-only: sim_recording setup ===

# === Review-only: CSV data logging ===

# === Main simulation loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

            frame += 1

        for _ in range(render_every):
            sim_time = system.GetChTime()

            # Override throttle with constant value per prompt requirement
            driver_inputs = driver.GetInputs()
            driver_inputs.m_throttle = CONSTANT_THROTTLE

            # Update sentinel and target marker positions each step
            sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
            target_marker.SetPos(steer_ctrl.GetTargetLocation())

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            hmmwv.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(time_step)
            terrain.Advance(time_step)
            hmmwv.Advance(time_step)
            vis.Advance(time_step)

            step_number += 1
            realtime_timer.Spin(time_step)

            # CSV logging — review-only per-line tagging

            if system.GetChTime() >= sim_end:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
