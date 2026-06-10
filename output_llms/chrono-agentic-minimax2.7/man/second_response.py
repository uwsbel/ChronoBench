"""
MAN 5t truck simulation on a rigid heightmap hill terrain.

System: NSC (wheeled vehicle wrapper, MAN_5t)
Vehicle: MAN 5t medium-duty truck, spawned at (-20, 0, 1.5)
Terrain: RigidTerrain with bump64.bmp heightmap patch (hills), grass texture
Driver: ChInteractiveDriverIRR (real-time interactive, review-only scripted maneuver)
Expected behavior: MAN 5t truck drives over hilly rigid terrain with grass texture.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import csv

import sim_recording as rec

# === Constants ===
step_size = 1e-3                              # physics time step (s)
sim_end = 20.0                                # simulation duration (s)
render_fps = 50.0                             # render at 50 fps
render_every = max(1, round(1.0 / (render_fps * step_size)))

TERRAIN_LENGTH = 64.0                         # bump64.bmp terrain length (m)
TERRAIN_WIDTH  = 64.0                         # bump64.bmp terrain width (m)
TERRAIN_HMIN   = 0.0                          # heightmap minimum height (m)
TERRAIN_HMAX   = 3.0                          # heightmap maximum height (m)

INIT_LOC = chrono.ChVector3d(-20, 0, 1.5)   # initial vehicle location per prompt
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)   # no rotation (facing +X)
track_point = chrono.ChVector3d(-3.0, 0.0, 1.1)  # chase camera target point

vis_type = veh.VisualizationType_MESH

# === Data paths (required for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
man = veh.MAN_5t()
man.SetContactMethod(chrono.ChContactMethod_NSC)
man.SetChassisCollisionType(veh.CollisionType_NONE)
man.SetChassisFixed(False)
man.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
man.SetTireType(veh.TireModelType_TMEASY)
man.SetTireStepSize(step_size)
man.Initialize()

# Visualization types set AFTER Initialize
man.SetChassisVisualizationType(vis_type)
man.SetSuspensionVisualizationType(vis_type)
man.SetSteeringVisualizationType(vis_type)
man.SetWheelVisualizationType(vis_type)
man.SetTireVisualizationType(vis_type)

# === System & bodies (created by the veh.MAN_5t wrapper) ===
sys = man.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", man.GetVehicle().GetMass())

# === Terrain — rigid hills via bump64 heightmap, grass texture ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
    TERRAIN_HMIN,
    TERRAIN_HMAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t on Rigid Hills")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(track_point, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(man.GetVehicle())

# === Driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_fps / steering_time)
driver.SetThrottleDelta(render_fps / throttle_time)
driver.SetBrakingDelta(render_fps / braking_time)
driver.Initialize()

# === Review-only recording setup ===
REC = bool(os.environ.get("SIMBENCH_RECORD"))


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

# cache: fetched once, reused every step
veh_obj = man.GetVehicle()
chassis = man.GetChassisBody()

try:
    while vis.Run() and sys.GetChTime() < sim_end:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        man.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)


        driver.Advance(step_size)
        terrain.Advance(step_size)
        man.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:
    import traceback; traceback.print_exc()
    raise
finally:
