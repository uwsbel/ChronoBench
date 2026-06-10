"""HMMWV ROS bridge demo using an NSC rigid-terrain vehicle system.

The simulation builds a full HMMWV vehicle, a flat rigid terrain patch with
defined contact material, a Chrono driver object, and ROS handlers for clock,
driver input subscription, and chassis state publication. The vehicle, terrain,
driver, visualizer, and ROS manager are synchronized and advanced each step.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Constants === define vehicle, terrain, and loop parameters once
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 6.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, math.ceil((1.0 / RENDER_FPS) / STEP_SIZE))  # precomputed once
TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT
TIRE_RADIUS = 0.47
WHEEL_Z_TOL = 0.12


# === Vehicle and system === full HMMWV wrapper owns the ChSystemNSC
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)  # prompt: explicit contact method
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  # prompt: explicit engine model
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # prompt: explicit tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned Chrono system reused throughout
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = hmmwv.GetVehicle()  # cache: underlying wheeled vehicle reused for ROS and visualization
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused for ROS and state logging
chassis.SetName("hmmwv_chassis")
print("VEHICLE MASS: ", vehicle.GetMass())

# wrapper-created components: chassis, suspension links, steering links, wheels,
# tires, and powertrain are owned by the HMMWV_Full wrapper and stepped by Advance().
spindle_positions = []
for axle_index in range(vehicle.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_positions.append(vehicle.GetSpindlePos(axle_index, side))
wheel_bottom_z = min(pos.z for pos in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -WHEEL_Z_TOL, (
    f"HMMWV wheel bottom z={wheel_bottom_z:.3f} is below rigid terrain beyond tolerance"
)

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain === rigid terrain patch supplies friction and restitution contacts
terrain = veh.RigidTerrain(system)
terrain_mat = chrono.ChContactMaterialNSC()
terrain_mat.SetFriction(TERRAIN_FRICTION)
terrain_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    terrain_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization === vehicle-aware Irrlicht window with chase camera and lighting
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV ROS Driver Inputs")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver and ROS === ROS subscribes to driver inputs and publishes vehicle state
driver = veh.ChDriver(vehicle)
driver.Initialize()

ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25.0, driver, "~/input/driver_inputs"))
ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, chassis, "~/output/hmmwv_chassis"))
ros_manager.Initialize()

# === Main loop === synchronize and advance vehicle, terrain, driver, and ROS
realtime_timer = chrono.ChRealtimeStepTimer()
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

            if not ros_manager.Update(time, STEP_SIZE):
                break


            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    raise RuntimeError(f"vehicle ROS simulation failed at step {step_number}") from exc
except (OSError, IOError) as exc:
    raise OSError("review recording file I/O failed") from exc
finally:
    pass
