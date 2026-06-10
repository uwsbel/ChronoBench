"""HMMWV rigid-highway simulation with an NSC rigid terrain patch.

The scene models a full HMMWV driving on a main rigid highway that intersects a
second rigid terrain patch.  The crossing patch uses friction 0.4 and
restitution 0.05, is centered at (6, -70, 0), and is rotated -90 degrees about
the world Z axis so the vehicle meets the terrain at the requested crossroads.
"""

import contextlib
import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === Named values keep terrain, vehicle, and recording parameters explicit.
TIME_STEP = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 6.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MAIN_ROAD_LENGTH = 120.0
MAIN_ROAD_WIDTH = 12.0
PATCH_LENGTH = 160.0
PATCH_WIDTH = 12.0
PATCH_THICKNESS = 0.04
PATCH_FRICTION = 0.4
PATCH_RESTITUTION = 0.05
PATCH_POS = chrono.ChVector3d(6.0, -70.0, 0.0)
PATCH_ROT = chrono.QuatFromAngleAxis(-chrono.CH_PI / 2.0, chrono.VECT_Z)

VEHICLE_START = chrono.ChVector3d(-8.0, -70.0, 0.5)
VEHICLE_HEADING = chrono.QUNIT
TIRE_RADIUS = 0.47
GROUND_Z = 0.0
Z_TOL = 0.08


# === Vehicle & terrain === The HMMWV wrapper owns the NSC system used by the rigid patch.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(VEHICLE_START, VEHICLE_HEADING))
hmmwv.SetTireType(veh.TireModelType_RIGID)  # rigid-highway tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: ChSystemNSC owned by the HMMWV wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: wrapper vehicle object reused for logging and visualization
chassis = hmmwv.GetChassisBody()  # cache: main chassis rigid body reused in the loop
print("VEHICLE MASS: ", vehicle.GetMass())
# Wrapper-created essentials: system, chassis, suspension links, steering links,
# wheels, tires, and powertrain are instantiated inside veh.HMMWV_Full.

spindle_positions = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= GROUND_Z - Z_TOL, (
    f"vehicle wheel bottom z={wheel_bottom_z:.3f} is below rigid patch z={GROUND_Z:.3f}"
)

terrain = veh.RigidTerrain(system)
base_mat = chrono.ChContactMaterialNSC()
base_mat.SetFriction(0.9)
base_mat.SetRestitution(0.01)
main_road = terrain.AddPatch(
    base_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(6.0, -70.0, -0.005), chrono.QUNIT),
    MAIN_ROAD_LENGTH,
    MAIN_ROAD_WIDTH,
)
main_road.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 80, 8)
main_road.SetColor(chrono.ChColor(0.48, 0.48, 0.48))

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(PATCH_FRICTION)
patch_mat.SetRestitution(PATCH_RESTITUTION)
terrain.Initialize()

patch = chrono.ChBodyEasyBox(
    PATCH_LENGTH,
    PATCH_WIDTH,
    PATCH_THICKNESS,
    1000.0,
    True,
    True,
    patch_mat,
)
patch.SetName("rotated_crossroads_terrain_patch")
patch.SetPos(PATCH_POS)
patch.SetRot(PATCH_ROT)
patch.SetFixed(True)
patch.GetVisualShape(0).SetColor(chrono.ChColor(0.62, 0.52, 0.36))
system.AddBody(patch)
system.GetCollisionSystem().BindAll()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Visualization === Vehicle-specific Irrlicht window follows the HMMWV.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid highway patch material and placement")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver === The scored core uses the standard interactive IRR driver.
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Main loop === Render at video cadence and advance all vehicle subsystems.
record_context = contextlib.nullcontext(None)

frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
try:
    with record_context as data_file:

        while vis.Run() and system.GetChTime() < SIM_END:

            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()


                driver.Synchronize(time)
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(TIME_STEP)
                terrain.Advance(TIME_STEP)
                hmmwv.Advance(TIME_STEP)
                vis.Advance(TIME_STEP)

                if system.GetChTime() >= SIM_END:
                    break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid Chrono state
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:  # output path or image-write failures
    traceback.print_exc()
    raise
finally:
    pass
