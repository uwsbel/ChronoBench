"""Kraz tractor-trailer on rigid terrain with a scripted double lane change.

This self-contained PyChrono 9.0 NSC vehicle scene starts the Kraz at
(-15, 0, 0.5), uses the Kraz tractor/trailer wrapper on a flat rigid road, and
drives a time-based lane-change maneuver through a Chrono data driver. The
Irrlicht chase camera tracks ahead of the vehicle with the requested long,
elevated chase distance.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# What/why: keep vehicle, terrain, driver, and render rates explicit and reused.
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 12.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 120.0
TERRAIN_WIDTH = 24.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_POS = chrono.ChVector3d(-15.0, 0.0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0.0)
CHASE_TARGET = chrono.ChVector3d(3.0, 0.0, 2.1)
CHASE_DISTANCE = 25.0
CHASE_HEIGHT = 10.5


# === Vehicle, system, and terrain ===
# What/why: use the catalog Kraz wrapper and its owned system for all subsystems.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisCollisionType(veh.CollisionType_NONE)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
kraz.SetTireStepSize(TIRE_STEP_SIZE)
kraz.Initialize()

system = kraz.GetSystem()  # cache: wrapper-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
tractor = kraz.GetTractor()  # cache: real tractor vehicle used by driver and visualizer
tractor_chassis = kraz.GetTractorChassisBody()  # cache: pose and velocity logging
trailer = kraz.GetTrailer()  # cache: confirms the trailer subsystem is present
print("VEHICLE MASS: ", tractor.GetMass())

# Wrapper-created components: Kraz owns the system, tractor body stack, trailer
# body stack, wheel/tire subsystems, powertrain, steering, and vehicle wrapper
# synchronization. Terrain, driver, visualization, and logging share this system.

kraz.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
kraz.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 80, 16)
patch.SetColor(chrono.ChColor(0.45, 0.47, 0.43))
terrain.Initialize()


# === Driver ===
# What/why: the prompt asks for a time-controlled double lane change maneuver.
driver_data = veh.vector_Entry()
driver_data.push_back(veh.DataDriverEntry(0.0, 0.00, 0.00, 0.0))
driver_data.push_back(veh.DataDriverEntry(0.5, 0.00, 0.45, 0.0))
driver_data.push_back(veh.DataDriverEntry(1.0, 0.00, 0.65, 0.0))
driver_data.push_back(veh.DataDriverEntry(2.0, 0.22, 0.70, 0.0))
driver_data.push_back(veh.DataDriverEntry(3.0, -0.28, 0.70, 0.0))
driver_data.push_back(veh.DataDriverEntry(4.0, -0.22, 0.70, 0.0))
driver_data.push_back(veh.DataDriverEntry(5.0, 0.28, 0.70, 0.0))
driver_data.push_back(veh.DataDriverEntry(6.0, 0.16, 0.65, 0.0))
driver_data.push_back(veh.DataDriverEntry(7.0, -0.12, 0.60, 0.0))
driver_data.push_back(veh.DataDriverEntry(8.0, 0.00, 0.55, 0.0))
driver_data.push_back(veh.DataDriverEntry(12.0, 0.00, 0.45, 0.0))
driver = veh.ChDataDriver(tractor, driver_data)
driver.Initialize()


# === Visualization ===
# What/why: vehicle-aware Irrlicht view with requested chase-camera target/distance.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetChaseCamera(CHASE_TARGET, CHASE_DISTANCE, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(tractor)


# === Main loop ===
# What/why: synchronize the complete vehicle subsystem stack at each physics step.
def run_simulation():
    frame = 0
    step_number = 0
    realtime_timer = chrono.ChRealtimeStepTimer()


    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver_inputs = driver.GetInputs()

                driver.Synchronize(time)
                terrain.Synchronize(time)
                kraz.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                kraz.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)

                step_number += 1
                realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:
        traceback.print_exc()
        raise
    finally:
        _ = step_number


if __name__ == "__main__":
    run_simulation()
