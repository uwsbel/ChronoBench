"""HMMWV rigid-highway vehicle scene with an added mesh bump terrain patch.

The simulation uses the catalog HMMWV_Full wrapper with an NSC contact method,
rigid tires, Bullet collision, and RigidTerrain patches.  A flat road patch
supports the vehicle while a bump.obj mesh patch at (0, -42, 0) provides the
requested additional terrain feature with blue-gray coloring and dirt texture.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants === named parameters keep vehicle, terrain, and recording reproducible
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

STEP_SIZE = 0.002
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

ROAD_LENGTH = 90.0
ROAD_WIDTH = 12.0
ROAD_CENTER = chrono.ChVector3d(0.0, -42.0, 0.0)
START_POS = chrono.ChVector3d(-36.0, -42.0, 0.5)
START_ROT = chrono.QUNIT
CHASE_TRACK = chrono.ChVector3d(0.0, 0.0, 1.75)


# === Vehicle system & bodies === wrapper owns the ChSystem and creates chassis, suspension, wheels, and tires
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(START_POS, START_ROT))
hmmwv.SetTireType(veh.TireModelType_RIGID)  # prompt/demo: rigid highway tire
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned ChSystem reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = hmmwv.GetChassisBody()  # cache: main chassis body logged every step
vehicle = hmmwv.GetVehicle()  # cache: full vehicle handle used by terrain, driver, visualization
print("VEHICLE MASS: ", vehicle.GetMass())

# wrapper-created components: chassis, suspension links, steering links, wheel bodies, tire models, and powertrain

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid road plus requested mesh bump patch using AddPatch
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
road_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(ROAD_CENTER, chrono.QUNIT),
    ROAD_LENGTH,
    ROAD_WIDTH,
)
road_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 18.0, 3.0)
road_patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

bump_patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, -42.0, 0.0), chrono.QUNIT),
    veh.GetDataFile("terrain/meshes/bump.obj"),
)
bump_patch.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
bump_patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)
terrain.Initialize()

spindle_positions = [
    vehicle.GetSpindlePos(axle, side)
    for axle in range(vehicle.GetNumberAxles())
    for side in (veh.LEFT, veh.RIGHT)
]
assert min(p.z for p in spindle_positions) > ROAD_CENTER.z, "wheel spindles must start above the rigid road"


# === Visualization & driver === Irrlicht vehicle visualizer mirrors catalog vehicle demos
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid Highway HMMWV")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_TRACK, 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetThrottleDelta((1.0 / RENDER_FPS) / 1.0)
driver.SetBrakingDelta((1.0 / RENDER_FPS) / 0.3)
driver.Initialize()


# === Main loop === synchronize and advance driver, terrain, vehicle, and visualization in order
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
step_number = 0
try:

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

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)
            vis.Advance(STEP_SIZE)

            step_number += 1
            if system.GetChTime() >= SIM_END:
                break
        realtime_timer.Spin(STEP_SIZE)
except (OSError, IOError) as exc:  # file-system errors while opening or writing review logs
    raise RuntimeError(f"review output failed: {exc}") from exc
except (RuntimeError, ValueError) as exc:  # Chrono solver/runtime state failures
    raise RuntimeError(f"simulation failed: {exc}") from exc
finally:
    pass
