"""HMMWV driving a circular path on flat rigid terrain (PyChrono 9.0.0, Irrlicht).

Models a full HMMWV wheeled vehicle (NSC contact) on a 200 x 200 m rigid-terrain
patch. Instead of a human-in-the-loop driver, an autonomous path-following driver
steers the vehicle around a circular Bezier path using a PID steering controller,
while the throttle is held at a constant 0.3 (no cruise-speed control). The circle
is drawn with two coloured balls (inner / outer markers), and the steering
controller's sentinel and target points are shown as live spheres so the path being
tracked is visible.

System type: ChSystemNSC (owned by the veh.HMMWV_Full wrapper).
Main bodies: HMMWV chassis + 4 wheels/tires, rigid terrain patch, path markers.
Expected behavior: the vehicle accelerates at constant throttle and the PID steering
controller keeps it circulating around the prescribed circle within the terrain.
"""

import os

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 2e-3                     # integration step (s)
tire_step_size = 1e-3                # tire force model sub-step (s)
sim_end = 18.0                       # bounded recording run (s)
render_fps = 50.0

terrain_length = 200.0               # X size (m) — enlarged so the circle fits
terrain_width = 200.0                # Y size (m)

path_radius = 30.0                   # circular path radius (m)
path_run_in = 10.0                   # straight run-in before the arc (m)
throttle_const = 0.3                 # constant throttle requested
init_height = 0.5                    # HMMWV chassis-origin height above ground at rest

# Derived render cadence (precomputed once)
render_steps = max(1, round(1.0 / (render_fps * step_size)))           # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth-faithful pair)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper (owns its ChSystemNSC) on rigid terrain
init_loc = chrono.ChVector3d(-path_run_in, 0, init_height)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # slip/grip tire for rigid road
hmmwv.SetTireStepSize(tire_step_size)
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = hmmwv.GetChassisBody()                      # cache: main chassis rigid body, reused
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)...; joints: suspension+steering inside wrapper
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain === single flat rigid patch sized to contain the circular path
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Path === circular Bezier path + two balls marking the circle
circle_path = veh.CirclePath(init_loc, path_radius, path_run_in, True, 5)

ball_mat = chrono.ChContactMaterialNSC()
ball_radius = 0.5
ball_center = chrono.ChVector3d(0, 0, init_height)    # circle centre (left turn from start)

inner_ball = chrono.ChBodyEasySphere(ball_radius, 1000, True, False, ball_mat)
inner_ball.SetPos(chrono.ChVector3d(ball_center.x, ball_center.y, ball_center.z))
inner_ball.SetFixed(True)
inner_ball.GetVisualShape(0).SetColor(chrono.ChColor(0.0, 0.8, 0.0))
system.AddBody(inner_ball)

outer_ball = chrono.ChBodyEasySphere(ball_radius, 1000, True, False, ball_mat)
outer_ball.SetPos(chrono.ChVector3d(ball_center.x, ball_center.y + path_radius, ball_center.z))
outer_ball.SetFixed(True)
outer_ball.GetVisualShape(0).SetColor(chrono.ChColor(0.0, 0.0, 0.8))
system.AddBody(outer_ball)

# === Driver === path-following PID steering; throttle forced constant (no speed control)
driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), circle_path, "circle_path", 1.0)
steer_ctrl = driver.GetSteeringController()           # cache: fetched once, reused every step
steer_ctrl.SetLookAheadDistance(5.0)
steer_ctrl.SetGains(0.8, 0.0, 0.0)                    # PID steering gains (KP, KI, KD)
driver.Initialize()

# === Path-point markers === sentinel + target spheres updated each frame
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_shape = chrono.ChVisualShapeSphere(0.25)
sentinel_shape.SetColor(chrono.ChColor(1.0, 0.0, 0.0))   # red sentinel (look-ahead point)
sentinel_marker.AddVisualShape(sentinel_shape, chrono.ChFramed())
system.AddBody(sentinel_marker)

target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_shape = chrono.ChVisualShapeSphere(0.25)
target_shape.SetColor(chrono.ChColor(1.0, 1.0, 0.0))     # yellow target (on-path point)
target_marker.AddVisualShape(target_shape, chrono.ChFramed())
system.AddBody(target_marker)

# === Visualization === full vehicle Irrlicht scene: window + sky + camera + lights
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.8)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Main loop === drive constant-throttle, PID-steered around the circle

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            sentinel_marker.SetPos(steer_ctrl.GetSentinelLocation())
            target_marker.SetPos(steer_ctrl.GetTargetLocation())

        driver_inputs = driver.GetInputs()
        driver_inputs.m_throttle = throttle_const     # constant throttle (override speed ctrl)
        driver_inputs.m_braking = 0.0

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
