"""Gator rigid-terrain multipatch simulation.

This PyChrono 9.0 NSC vehicle scene models a John Deere Gator crossing four
rigid terrain patches with distinct visual textures. One patch is initialized
from a height map for grade testing, and fixed bump bodies provide local
obstacles on the patch sequence. The expected behavior is a stable vehicle
that can be driven over flat, textured, graded, and bumped terrain.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
PATCH_LENGTH = 18.0
PATCH_WIDTH = 8.0
PATCH_THICKNESS = 0.35
PATCH_Z = -0.18
BUMP_LENGTH = 0.45
BUMP_WIDTH = PATCH_WIDTH * 0.82
BUMP_HEIGHT = 0.22
GATOR_INIT_X = -30.0
GATOR_INIT_Y = 0.0
GATOR_INIT_Z = 0.55


# === Paths & materials ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

terrain_textures = [
    veh.GetDataFile("terrain/textures/grass.jpg"),
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    veh.GetDataFile("terrain/textures/Gravel034_1K-JPG/Gravel034_1K_Color.jpg"),
    veh.GetDataFile("terrain/textures/dirt.jpg"),
]
heightmap_file = veh.GetDataFile("terrain/height_maps/slope.bmp")

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.85)
patch_mat.SetRestitution(0.01)

bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.9)
bump_mat.SetRestitution(0.02)


# === Vehicle & system ===
vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(GATOR_INIT_X, GATOR_INIT_Y, GATOR_INIT_Z),
        chrono.QUNIT,
    )
)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body reused for camera tracking/logging
veh_obj = vehicle.GetVehicle()  # cache: underlying vehicle handle for visualization/diagnostics
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain patches & bumps ===
terrain = veh.RigidTerrain(system)
patch_centers = [-27.0, -9.0, 9.0, 27.0]

for index, center_x in enumerate(patch_centers):
    patch_frame = chrono.ChCoordsysd(
        chrono.ChVector3d(center_x, 0.0, PATCH_Z),
        chrono.QUNIT,
    )
    if index == 2:
        patch = terrain.AddPatch(
            patch_mat,
            patch_frame,
            heightmap_file,
            PATCH_LENGTH,
            PATCH_WIDTH,
            -0.10,
            1.15,
        )
    else:
        patch = terrain.AddPatch(
            patch_mat,
            patch_frame,
            PATCH_LENGTH,
            PATCH_WIDTH,
            PATCH_THICKNESS,
        )
    patch.SetTexture(terrain_textures[index], 8.0, 8.0)

    bump = chrono.ChBodyEasyBox(
        BUMP_LENGTH,
        BUMP_WIDTH,
        BUMP_HEIGHT,
        1000.0,
        True,
        True,
        bump_mat,
    )
    bump.SetName(f"speed_bump_patch_{index + 1}")
    bump.SetFixed(True)
    bump.SetPos(chrono.ChVector3d(center_x + 3.0, 0.0, BUMP_HEIGHT * 0.5))
    bump.EnableCollision(True)
    system.AddBody(bump)

terrain.Initialize()

spindle_positions = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle, side))
min_spindle_z = min(pos.z for pos in spindle_positions)
assert min_spindle_z > 0.25, "Gator spindle height is too low for the rigid terrain patches"


# === Visualization & driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator multipatch terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.8), 8.0, 0.4)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(0.04)
driver.SetThrottleDelta(0.04)
driver.SetBrakingDelta(0.08)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:

    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        vehicle.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:
    print(f"vehicle-terrain simulation failed: {exc}")
    raise
except (OSError, IOError) as exc:
    print(f"vehicle visualization I/O failed: {exc}")
    raise
finally:
    pass
