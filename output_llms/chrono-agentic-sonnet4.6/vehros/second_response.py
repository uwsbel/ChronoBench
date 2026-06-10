"""
HMMWV Vehicle with Irrlicht Visualization and ROS2 Bridge.

Models an HMMWV_Full wheeled vehicle on a flat RigidTerrain (NSC contact)
with full Irrlicht visualization (sky, camera, typical lights, shadow light)
and a ChROSPythonManager that:
  - publishes /clock (ChROSClockHandler) for ROS time synchronization,
  - subscribes driver inputs at ~/input/driver_inputs (ChROSDriverInputsHandler),
  - publishes chassis pose/twist at ~/output/hmmwv/state (ChROSBodyHandler).

System: ChSystemNSC owned by HMMWV_Full wrapper.
Bodies: HMMWV chassis + suspension/steering/wheel sub-bodies on rigid flat terrain.
Expected: Vehicle rests on flat terrain; ROS graph receives /clock and vehicle
          state at 25 Hz; subscribed driver inputs are applied each step.
"""

# === Imports ===
import math
import os
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr
import pychrono.ros as chros


# === Simulation parameters ===
TIME_STEP = 1e-3        # physics step size (s)
SIM_END = 30.0          # simulation end time (s)
RENDER_FPS = 25.0       # Irrlicht render rate (Hz)
TERRAIN_LENGTH = 100.0  # terrain patch length (m)
TERRAIN_WIDTH = 100.0   # terrain patch width (m)
DRIVER_RATE = 25        # ROS handler publish/subscribe rate (Hz)

# Precomputed once: physics steps per rendered frame
render_steps = math.ceil(1.0 / (RENDER_FPS * TIME_STEP))  # precomputed once

# === Data paths (required truth components for catalog vehicles) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                              # MANDATORY: fixed chassis won't move
hmmwv.SetInitPosition(
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.6), chrono.ChQuaterniond(1, 0, 0, 0))
)
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# === Visualization types (set after Initialize) ===
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()             # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()       # cache: main chassis rigid body; fetched once
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i)...; created inside the HMMWV_Full wrapper
# joints: suspension + steering links created inside the HMMWV_Full wrapper

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# === Irrlicht visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain")
vis.Initialize()                                           # Initialize FIRST on Irrlicht
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Driver (ChDriver — base driver for ROS-driven vehicle control) ===
driver = veh.ChDriver(hmmwv.GetVehicle())
driver.Initialize()

# === ROS2 manager + handlers ===
ros_manager = chros.ChROSPythonManager()
# Clock handler first — publishes /clock for ROS time synchronization
ros_manager.RegisterHandler(chros.ChROSClockHandler())
# Driver-inputs handler — SUBSCRIBES to steering/throttle/brake from ROS
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(DRIVER_RATE, driver, "~/input/driver_inputs")
)
# Body handler — publishes chassis pose/twist to ROS
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(DRIVER_RATE, chassis, "~/output/hmmwv/state")
)
# Initialize exactly once, after all handlers are registered
ros_manager.Initialize()

# === Review-only: recording setup ===

# === Main loop ===
step_number = 0
frame = 0  # review-only: consecutive frame index

# Enable real-time simulation for the vehicle
hmmwv.GetVehicle().EnableRealtime(True)

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)       # advances the wrapper-owned ChSystem

        # ROS update last — after all state-producing updates
        if not ros_manager.Update(sim_time, TIME_STEP):
            break


        step_number += 1

        if system.GetChTime() >= SIM_END:
            break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
