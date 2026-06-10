"""HMMWV rigid-terrain multipatch demo using NSC contact.

The scene contains a full HMMWV vehicle on four rigid terrain patches: two box
patches, one mesh patch, and one heightmap patch. The patches are placed at the
requested final coordinates so the vehicle can drive over separated terrain
surfaces while the Irrlicht visualizer shows the complete environment.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named simulation and terrain parameters for reproducibility
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

PATCH_LENGTH = 32.0
PATCH_WIDTH = 16.0
PATCH_THICKNESS = 0.2
PATCH1_POS = chrono.ChVector3d(-20.0, 5.0, 0.0)
PATCH2_POS = chrono.ChVector3d(20.0, -5.0, 0.2)
PATCH3_POS = chrono.ChVector3d(5.0, -45.0, 0.0)
PATCH4_POS = chrono.ChVector3d(10.0, 40.0, 0.0)
VEH_INIT_POS = chrono.ChVector3d(-34.0, 5.0, 0.55)
VEH_INIT_ROT = chrono.QuatFromAngleZ(0.0)


# === Vehicle and system === wrapper creates the vehicle bodies and owned NSC system
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(VEH_INIT_POS, VEH_INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()  # cache: wrapper-owned ChSystem reused for terrain and logs
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = vehicle.GetChassisBody()  # cache: chassis body reused in logging and assertions
veh_obj = vehicle.GetVehicle()  # cache: low-level vehicle handle reused for mass and spindles
# wrapper-created bodies: chassis, suspensions, steering, wheels, tires; joints are created by HMMWV_Full
print("VEHICLE MASS: ", veh_obj.GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain patches === four rigid patches keep box, mesh, and heightmap forms
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch1 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH1_POS, chrono.QUNIT),
    PATCH_LENGTH,
    PATCH_WIDTH,
    PATCH_THICKNESS,
)
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 10, 6)
patch1.SetColor(chrono.ChColor(0.75, 0.75, 0.65))

patch2 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH2_POS, chrono.QUNIT),
    PATCH_LENGTH,
    PATCH_WIDTH,
    PATCH_THICKNESS,
)
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 10, 6)
patch2.SetColor(chrono.ChColor(0.65, 0.65, 0.65))

patch3 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH3_POS, chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
    True,
    0.01,
)
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)
patch3.SetColor(chrono.ChColor(0.55, 0.42, 0.28))

patch4 = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(PATCH4_POS, chrono.QUNIT),
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),
    PATCH_LENGTH,
    PATCH_LENGTH,
    -0.25,
    0.65,
    True,
    0.01,
)
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 8, 8)
patch4.SetColor(chrono.ChColor(0.45, 0.65, 0.35))

terrain.Initialize()

spindle_positions = []
for axle_index in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(veh_obj.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - 0.47
assert wheel_bottom_z > -0.10, f"vehicle wheel bottom starts too low: {wheel_bottom_z:.3f} m"


# === Visualization === vehicle-specific Irrlicht window with sky and directional light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid Terrain Multipatches")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(veh_obj)


# === Driver === interactive vehicle driver matching the real-time catalog demos
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0


# === Main loop === synchronized driver, terrain, vehicle, and visual system updates
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            driver_inputs = driver.GetInputs()  # cache: one input fetch per dynamics step

            driver.Synchronize(time)
            terrain.Synchronize(time)
            vehicle.Synchronize(time, driver_inputs, terrain)
            vis.Synchronize(time, driver_inputs)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            vehicle.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            if system.GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(STEP_SIZE)
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
