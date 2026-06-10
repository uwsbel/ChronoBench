"""
HMMWV Circular Path Following Simulation (PyChrono 9.0.x, Irrlicht, NSC)

Models a full HMMWV vehicle following a circular path using ChPathFollowerDriver
with a PID steering controller and constant throttle of 0.3.

System: ChSystemNSC (owned by HMMWV_Full wrapper)
Terrain: RigidTerrain flat patch, 200 m x 200 m
Driver: ChPathFollowerDriver — circular path, radius 40 m, speed 8 m/s
Expected behavior: HMMWV drives in continuous circles; sentinel (blue sphere) and
target (red sphere) markers track the steering controller look-ahead points.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# === Data paths (mandatory truth components for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
step_size           = 1e-3       # physics time step (s)
sim_end             = 30.0       # simulation end time (s)
render_fps          = 50.0       # Irrlicht render rate (Hz)
render_steps        = max(1, math.ceil((1.0 / render_fps) / step_size))  # precomputed once

TERRAIN_LENGTH      = 200.0      # terrain length X (m) — increased to fit circular path
TERRAIN_WIDTH       = 200.0      # terrain width Y (m)
CIRCLE_RADIUS       = 40.0       # circular path radius (m)
CIRCLE_CENTER_X     = 0.0        # circle centre X
CIRCLE_CENTER_Y     = 0.0        # circle centre Y
CIRCLE_RUN_IN       = 5.0        # straight run-in before the circle (m)
TARGET_SPEED        = 8.0        # cruise speed for the path follower (m/s)
THROTTLE_CONST      = 0.3        # constant throttle — overrides speed controller below
SUSPENSION_REF_HEIGHT = 0.5      # chassis origin above wheel-bottom at rest (HMMWV)
TIRE_RADIUS         = 0.47       # approximate HMMWV tire radius (m) — for footprint assert

# === Vehicle init position — start at path origin ===
init_x   = CIRCLE_CENTER_X + CIRCLE_RADIUS   # path start (east of centre)
init_y   = CIRCLE_CENTER_Y
init_z   = SUSPENSION_REF_HEIGHT
init_loc = chrono.ChVector3d(init_x, init_y, init_z)
init_rot = chrono.QuatFromAngleZ(math.pi / 2)   # face north (Y direction) at circle start

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system  = hmmwv.GetSystem()                     # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()                # cache: fetched once, reused in path marker
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i); terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

# Set visualization types (called after Initialize)
hmmwv.SetChassisVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(chrono.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(chrono.VisualizationType_MESH)

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# Verify wheel footprint
ZTOL = 0.10
veh_obj = hmmwv.GetVehicle()
spindle_zvals = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        try:
            p = veh_obj.GetSpindlePos(axle_idx, side)
            spindle_zvals.append(p.z)
        except (RuntimeError, AttributeError):
            pass
if spindle_zvals:
    wheel_bottom_z = min(spindle_zvals) - TIRE_RADIUS
    assert wheel_bottom_z >= -ZTOL, (
        f"vehicle sinks: wheel_bottom_z={wheel_bottom_z:.3f}; "
        f"raise SUSPENSION_REF_HEIGHT by {-wheel_bottom_z:.3f} m"
    )

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path — circular with straight run-in ===
path_start = chrono.ChVector3d(init_x, init_y, 0.5)
path = veh.CirclePath(path_start, CIRCLE_RADIUS, CIRCLE_RUN_IN, True, 3)

# === Path visualization — two fixed balls along the path ===
ball1_body = chrono.ChBody()
ball1_body.SetFixed(True)
ball1_body.SetPos(chrono.ChVector3d(CIRCLE_CENTER_X + CIRCLE_RADIUS, CIRCLE_CENTER_Y, 0.5))
ball1_vis = chrono.ChVisualShapeSphere(0.5)
ball1_vis.SetColor(chrono.ChColor(1.0, 1.0, 0.0))   # yellow
ball1_body.AddVisualShape(ball1_vis, chrono.ChFramed())
ball1_body.SetName("path_marker_1")
system.AddBody(ball1_body)

ball2_body = chrono.ChBody()
ball2_body.SetFixed(True)
ball2_body.SetPos(chrono.ChVector3d(CIRCLE_CENTER_X - CIRCLE_RADIUS, CIRCLE_CENTER_Y, 0.5))
ball2_vis = chrono.ChVisualShapeSphere(0.5)
ball2_vis.SetColor(chrono.ChColor(0.0, 1.0, 1.0))   # cyan
ball2_body.AddVisualShape(ball2_vis, chrono.ChFramed())
ball2_body.SetName("path_marker_2")
system.AddBody(ball2_body)

# === Path follower driver (replaces interactive driver — circular path + PID steering) ===
driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), path, "circle_path", TARGET_SPEED)
driver.GetSteeringController().SetLookAheadDistance(5.0)
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)   # KP, KI, KD (PID steering)
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# === Sentinel and target point markers (fixed bodies updated each frame) ===
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_marker.SetPos(chrono.ChVector3d(init_x, init_y, 0.5))
s_sphere = chrono.ChVisualShapeSphere(0.3)
s_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))   # blue — sentinel
sentinel_marker.AddVisualShape(s_sphere, chrono.ChFramed())
sentinel_marker.SetName("sentinel_marker")
system.AddBody(sentinel_marker)

target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_marker.SetPos(chrono.ChVector3d(init_x, init_y, 0.5))
t_sphere = chrono.ChVisualShapeSphere(0.3)
t_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))   # red — target
target_marker.AddVisualShape(t_sphere, chrono.ChFramed())
target_marker.SetName("target_marker")
system.AddBody(target_marker)

# === Visualization (Irrlicht) — Initialize FIRST, then add scene elements ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Following")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()                                          # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                 # vehicle truth uses directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Recording setup (review-only) ===


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number    = 0

try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        # Throttled rendering
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs and override throttle to constant 0.3
        driver_inputs = driver.GetInputs()
        driver_inputs.m_throttle = THROTTLE_CONST   # constant throttle as specified

        # Update sentinel and target markers to current controller look-ahead positions
        steer_ctrl = driver.GetSteeringController()   # cache: fetched once per frame
        sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
        target_marker.SetPos(steer_ctrl.GetTargetLocation())

        # Synchronize subsystems (driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        # Advance subsystems
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:    # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
