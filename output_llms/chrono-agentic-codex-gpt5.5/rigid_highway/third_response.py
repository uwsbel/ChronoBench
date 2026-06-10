"""HMMWV on a rigid highway terrain patch using NSC contact.

The scene builds a full HMMWV vehicle, a single rigid terrain patch with the
requested contact material, and an Irrlicht vehicle visualizer. The patch is
rotated -90 degrees about Z and moved to (6, -70, 0), placing the vehicle at a
cross-road style intersection with the road surface.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 300.0
TERRAIN_WIDTH = 16.0
BASE_PATCH_CENTER = chrono.ChVector3d(0.0, 0.0, 0.0)
BASE_PATCH_FRAME = chrono.ChCoordsysd(BASE_PATCH_CENTER, chrono.QUNIT)
PATCH_CENTER = chrono.ChVector3d(6.0, -70.0, 0.0)
PATCH_ROT = chrono.QuatFromAngleAxis(-math.pi / 2.0, chrono.ChVector3d(0, 0, 1))
PATCH_FRAME = chrono.ChCoordsysd(PATCH_CENTER, PATCH_ROT)

VEHICLE_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.5)
VEHICLE_INIT_ROT = chrono.QUNIT


# === Vehicle and terrain ===
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

system = hmmwv.GetSystem()  # cache: vehicle-owned system reused by terrain and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

chassis = hmmwv.GetChassisBody()  # cache: used by visualization and data logging
vehicle_core = hmmwv.GetVehicle()  # cache: exposes mass and speed diagnostics

# Wrapper-created essentials: system, chassis, suspension links, spindles,
# wheels, tires, powertrain, visualization, driver, and terrain all share system.
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.4)
patch_mat.SetRestitution(0.05)
base_patch = terrain.AddPatch(patch_mat, BASE_PATCH_FRAME, TERRAIN_LENGTH, TERRAIN_WIDTH)
base_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 300.0, 16.0)
base_patch.SetColor(chrono.ChColor(0.42, 0.42, 0.42))
cross_patch = terrain.AddPatch(patch_mat, PATCH_FRAME, TERRAIN_LENGTH, TERRAIN_WIDTH)
cross_patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 300.0, 16.0)
cross_patch.SetColor(chrono.ChColor(0.42, 0.42, 0.42))
terrain.Initialize()


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Rigid highway terrain patch")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 24.0, 8.0)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle_core)

driver_data = veh.vector_Entry(
    [
        veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
        veh.DataDriverEntry(0.5, 0.0, 0.4, 0.0),
        veh.DataDriverEntry(SIM_END, 0.0, 0.4, 0.0),
    ]
)
driver = veh.ChDataDriver(vehicle_core, driver_data)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
def run_simulation() -> None:
    """Run the vehicle scene and record review artifacts when requested."""
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
                if step_number % 10 == 0:
                    realtime_timer.Spin(STEP_SIZE)
                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError, OSError, IOError) as exc:
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    run_simulation()
