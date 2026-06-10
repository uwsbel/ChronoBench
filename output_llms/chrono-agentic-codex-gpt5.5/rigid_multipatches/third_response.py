"""HMMWV rigid-terrain multipatch demo using NSC contact.

The simulation builds a wrapper-owned HMMWV_Full vehicle on four rigid terrain
patches: a mesh bump, a height-map bump, and two flat box patches.  The patch
centers are the requested final positions, and the vehicle/terrain/driver stack
is advanced through the standard Chrono vehicle synchronization order.
"""

import math
import os
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_STEP_SIZE = 1.0 / RENDER_FPS
RENDER_STEPS = max(1, math.ceil(RENDER_STEP_SIZE / STEP_SIZE))  # precomputed once

PATCH_HALF_TOL = 0.08
HMMWV_TIRE_RADIUS = 0.467
VEHICLE_INIT_POS = chrono.ChVector3d(5.0, -45.0, 0.62)
VEHICLE_INIT_ROT = chrono.QuatFromAngleAxis(chrono.CH_PI / 2, chrono.VECT_Z)

PATCH1_POS = chrono.ChVector3d(-20.0, 5.0, 0.0)
PATCH2_POS = chrono.ChVector3d(20.0, -5.0, 0.2)
PATCH3_POS = chrono.ChVector3d(5.0, -45.0, 0.0)
PATCH4_POS = chrono.ChVector3d(10.0, 40.0, 0.0)

PATCH1_SIZE = (30.0, 30.0)
PATCH2_SIZE = (30.0, 30.0)
PATCH3_SIZE = (25.0, 25.0)
PATCH4_SIZE = (25.0, 25.0)


# === Vehicle and system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEHICLE_INIT_POS, VEHICLE_INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle = hmmwv.GetVehicle()  # cache: wrapper vehicle handle reused every step
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for diagnostics
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials: system, chassis/suspension/wheels/tires, terrain,
# Irrlicht vehicle visualizer, and interactive driver all share this vehicle system.


# === Terrain patches ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH1_POS, chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 8.0, 8.0)
patch1.SetColor(chrono.ChColor(0.55, 0.55, 0.50))

patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH2_POS, chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    PATCH2_SIZE[0],
    PATCH2_SIZE[1],
    -0.7,
    0.7,
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 8.0, 8.0)
patch2.SetColor(chrono.ChColor(0.48, 0.34, 0.20))

patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH3_POS, chrono.QUNIT),
    PATCH3_SIZE[0],
    PATCH3_SIZE[1],
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 10.0, 10.0)
patch3.SetColor(chrono.ChColor(0.45, 0.50, 0.55))

patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH4_POS, chrono.QUNIT),
    PATCH4_SIZE[0],
    PATCH4_SIZE[1],
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10.0, 10.0)
patch4.SetColor(chrono.ChColor(0.70, 0.70, 0.55))

terrain.Initialize()

spindle_positions = []  # cache: immediate footprint check after vehicle creation
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - HMMWV_TIRE_RADIUS
assert wheel_bottom_z >= PATCH3_POS.z - PATCH_HALF_TOL, (
    f"vehicle sinks into spawn patch: wheel bottom z={wheel_bottom_z:.3f}, "
    f"patch top z={PATCH3_POS.z:.3f}"
)


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid Terrain Multipatches")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_STEP_SIZE / 0.3)
driver.Initialize()


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()

        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise RuntimeError("failed to write review output files") from exc
except (RuntimeError, ValueError, AssertionError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
