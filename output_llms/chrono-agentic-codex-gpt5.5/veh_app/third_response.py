"""HMMWV vehicle application with a chassis-mounted depth camera.

This NSC vehicle simulation drives a full HMMWV over rigid terrain, attaches a
depth camera to the chassis at the requested offset, and logs the vehicle
position and heading at every dynamics step.
"""

import csv
import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 0.002
TIRE_STEP_SIZE = 0.002
SIM_END = 1.0
RENDER_FPS = 10.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 80.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT

DEPTH_UPDATE_RATE = 30.0
DEPTH_WIDTH = 1280
DEPTH_HEIGHT = 720
DEPTH_HFOV = 1.408
DEPTH_MAX_DISTANCE = 30.0
DEPTH_OFFSET = chrono.ChVector3d(-5.0, 0.0, 2.0)
ENABLE_DEPTH_VISUALIZER = True
ENABLE_SENSOR_UPDATES = True


# === Vehicle and terrain ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned physical system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

vehicle_core = hmmwv.GetVehicle()  # cache: repeated spindle and mass access
chassis = hmmwv.GetChassisBody()  # cache: sensor parent and pose logging body
spindle_world = []
for axle_index in range(vehicle_core.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(vehicle_core.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(point.z for point in spindle_world) - 0.47
assert wheel_bottom_z >= -0.1, (
    f"vehicle wheel bottom is below the terrain plane: {wheel_bottom_z:.3f}"
)

# Wrapper-created components are the HMMWV vehicle system, chassis, suspension,
# steering, wheel/tire subsystems, rigid terrain, Irrlicht visualizer, driver, and
# chassis-mounted depth camera.


# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Depth Camera Vehicle Application")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_FPS * STEP_SIZE / 1.0)
driver.SetThrottleDelta(RENDER_FPS * STEP_SIZE / 1.0)
driver.SetBrakingDelta(RENDER_FPS * STEP_SIZE / 0.3)
driver.Initialize()


# === Depth camera sensor ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-8.0, -3.0, 20.0),
    chrono.ChColor(0.8, 0.8, 0.8),
    120.0,
)

depth_pose = chrono.ChFramed(DEPTH_OFFSET, chrono.QUNIT)
depth_camera = sens.ChDepthCamera(
    chassis,
    DEPTH_UPDATE_RATE,
    depth_pose,
    DEPTH_WIDTH,
    DEPTH_HEIGHT,
    DEPTH_HFOV,
    DEPTH_MAX_DISTANCE,
)
depth_camera.SetName("Depth Camera")
depth_camera.SetLag(0)
depth_camera.SetCollectionWindow(0)
depth_camera.SetMaxDepth(DEPTH_MAX_DISTANCE)
depth_camera.PushFilter(sens.ChFilterDepthAccess())
depth_camera.PushFilter(sens.ChFilterDepthToRGBA8())
if ENABLE_DEPTH_VISUALIZER:
    depth_camera.PushFilter(sens.ChFilterVisualize(DEPTH_WIDTH, DEPTH_HEIGHT, "Depth Map"))
if ENABLE_SENSOR_UPDATES:
    manager.AddSensor(depth_camera)


# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    with open("vehicle_state.csv", "w", newline="") as state_file:
        writer = csv.writer(state_file)
        writer.writerow(["time", "x", "y", "z", "heading"])

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
                if ENABLE_SENSOR_UPDATES:
                    manager.Update()

                pos = chassis.GetPos()
                heading = chassis.GetRot().GetCardanAnglesZYX().z
                writer.writerow([system.GetChTime(), pos.x, pos.y, pos.z, heading])

                if ENABLE_SENSOR_UPDATES:
                    depth_buffer = depth_camera.GetMostRecentDepthBuffer()
                    if depth_buffer.HasData():
                        pass

                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break

except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
finally:
    print("Vehicle state logging flushed.")
