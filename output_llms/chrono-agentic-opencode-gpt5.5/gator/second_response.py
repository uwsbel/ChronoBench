"""Gator vehicle on four terrain patches.

This PyChrono 9.0.0 NSC simulation places a Gator utility vehicle on four
adjacent rigid terrain patches with distinct textures. One patch uses a bundled
height map for gradability testing, and each patch includes a visible speed-bump
obstacle so the vehicle contacts varied terrain features while driving forward.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named values keep geometry and timing easy to audit
TIME_STEP = 1.0e-3
TIRE_STEP = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

PATCH_LENGTH = 10.0
PATCH_WIDTH = 8.0
PATCH_CENTERS_X = (-15.0, -5.0, 5.0, 15.0)
PATCH_CENTER_Y = 0.0
GATOR_START = chrono.ChVector3d(-19.0, 0.0, 0.42)
BUMP_RADIUS = 0.22
BUMP_WIDTH = 6.0


# === Vehicle and system === catalog Gator owns its NSC system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.Gator()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(GATOR_START, chrono.QUNIT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystemNSC reused below
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: main chassis body reused for logging
veh_model = vehicle.GetVehicle()  # cache: vehicle subsystem queried repeatedly
# Wrapper-created components are explicit here: system, chassis, suspension joints,
# steering links, wheels, tires, and drivetrain are owned by veh.Gator().
print("VEHICLE MASS: ", veh_model.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain and bumps === four rigid patches, including one height-map grade patch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
textures = (
    "terrain/textures/tile4.jpg",
    "terrain/textures/dirt.jpg",
    "terrain/textures/grass.jpg",
    "terrain/textures/concrete.jpg",
)

for index, center_x in enumerate(PATCH_CENTERS_X):
    patch_frame = chrono.ChCoordsysd(chrono.ChVector3d(center_x, PATCH_CENTER_Y, 0.0), chrono.QUNIT)
    if index == 2:
        patch = terrain.AddPatch(
            patch_mat,
            patch_frame,
            veh.GetDataFile("terrain/height_maps/bump64.bmp"),
            PATCH_LENGTH,
            PATCH_WIDTH,
            -0.15,
            0.75,
        )
    else:
        patch = terrain.AddPatch(patch_mat, patch_frame, PATCH_LENGTH, PATCH_WIDTH)
    patch.SetTexture(veh.GetDataFile(textures[index]), 4.0, 4.0)

terrain.Initialize()

bump_mat = chrono.ChContactMaterialNSC()
bump_mat.SetFriction(0.8)
bump_mat.SetRestitution(0.02)
for index, center_x in enumerate(PATCH_CENTERS_X):
    bump_x = center_x + (-2.0 if index < 2 else 2.0)
    bump = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, BUMP_RADIUS, BUMP_WIDTH, 1000.0, True, True, bump_mat)
    bump.SetName(f"terrain_patch_{index + 1}_speed_bump")
    bump.SetPos(chrono.ChVector3d(bump_x, 0.0, BUMP_RADIUS))
    bump.SetFixed(True)
    bump.EnableCollision(True)
    system.AddBody(bump)

system.GetCollisionSystem().BindAll()

spindle_positions = []
for axle_i in range(veh_model.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_model.GetSpindlePos(axle_i, side))
assert min(p.z for p in spindle_positions) > 0.15, "Gator spindle height indicates terrain spawn overlap"


# === Visualization and driver === Irrlicht vehicle view with standard directional light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator multi-patch terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.8), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_model)

driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 1.0),
    veh.DataDriverEntry(0.4, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.8, 0.0, 0.65, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, 0.65, 0.0),
])
driver = veh.ChDataDriver(veh_model, driver_data)
driver.Initialize()


# === Main loop === scored-core real-time vehicle synchronization and advance
realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(RENDER_EVERY):
        time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)
        if system.GetChTime() >= SIM_END:
            break

    realtime_timer.Spin(TIME_STEP)
