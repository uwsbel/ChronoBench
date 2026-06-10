"""HMMWV rigid-highway simulation on NSC terrain with an added mesh bump patch.

The scene uses a wrapper-managed HMMWV full vehicle, rigid terrain contact, and
an Irrlicht vehicle visual system.  A flat highway patch supports the vehicle,
and a requested bump.obj mesh patch is placed at (0, -42, 0) with the specified
blue-gray color and dirt texture so the vehicle can drive across it.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Vehicle, terrain, and run constants are named so the setup is self-contained.
STEP_SIZE = 2.0e-3
SIM_END = 12.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

INIT_POS = chrono.ChVector3d(0.0, -82.0, 0.55)
INIT_ROT = chrono.QuatFromAngleZ(chrono.CH_PI / 2.0)
CHASE_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)

FLAT_PATCH_CENTER = chrono.ChVector3d(0.0, -42.0, 0.0)
FLAT_PATCH_LENGTH = 18.0
FLAT_PATCH_WIDTH = 300.0
BUMP_PATCH_CENTER = chrono.ChVector3d(0.0, -42.0, 0.0)

FRICTION = 0.9
RESTITUTION = 0.01
TIRE_STEP_SIZE = STEP_SIZE


# === Vehicle data paths ===
# Bundled vehicle assets are resolved from the Chrono data tree.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


# === Vehicle system ===
# The HMMWV wrapper owns the ChSystem; terrain and visualization attach to it.
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: vehicle-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

vehicle_core = hmmwv.GetVehicle()  # cache: queried for mass, footprint, sync
chassis = hmmwv.GetChassisBody()  # cache: physical protagonist body


# === Terrain patches ===
# RigidTerrain owns the flat highway and the requested mesh bump patch.
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(FRICTION)
patch_mat.SetRestitution(RESTITUTION)

terrain = veh.RigidTerrain(system)
flat_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(FLAT_PATCH_CENTER, chrono.QUNIT),
    FLAT_PATCH_LENGTH,
    FLAT_PATCH_WIDTH,
)
flat_patch.SetColor(chrono.ChColor(0.45, 0.45, 0.45))
flat_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 12.0, 48.0)

bump_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(BUMP_PATCH_CENTER, chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)

terrain.Initialize()


# === Footprint check ===
# Verify the initialized wheels start on top of the rigid support.
spindle_world = []  # cache: initialized wheel centers for support sanity check
for axle_index in range(vehicle_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle_core.GetSpindlePos(axle_index, side))

first_tire = vehicle_core.GetAxles()[0].m_wheels[0].GetTire()
tire_radius = first_tire.GetRadius()  # cache: wheel-bottom calculation
wheel_bottom_z = min(pos.z for pos in spindle_world) - tire_radius
assert wheel_bottom_z >= -0.08, (
    f"vehicle starts below terrain: wheel bottom z={wheel_bottom_z:.3f}"
)


# === Visualization ===
# Vehicle-specific Irrlicht visualization follows the wrapper truth pattern.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid Highway")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_POINT, 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)


# === Driver ===
# A scripted driver keeps the unattended highway validation moving over the bump.
class HighwayDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.35:
            self.SetThrottle(0.0)
            self.SetBraking(0.15)
        else:
            self.SetThrottle(0.9)
            self.SetBraking(0.0)
        self.SetSteering(0.0)


driver = HighwayDriver(vehicle_core)
driver.Initialize()


# === Review output setup ===


# === Main loop ===
# Real-time wrapper loop synchronizes driver, terrain, vehicle, and visual system.
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()

            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)


            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError, AssertionError) as exc:
    print(f"simulation failed: {exc}")
    raise
finally:
    pass


# === Review post-processing ===
