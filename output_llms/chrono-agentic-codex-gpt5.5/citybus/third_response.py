"""CityBus rigid-terrain PyChrono simulation.

This NSC vehicle scene runs a Chrono CityBus on a flat rigid dirt road using
Pacejka 89 tires, 5e-4 second vehicle and tire steps, and the standard
Irrlicht vehicle visualizer. The bus is initialized free to move and is
controlled through the catalog interactive driver in the scored core.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# === Constants ===
STEP_SIZE = 5e-4
TIRE_STEP_SIZE = 5e-4
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, math.ceil(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
TRACK_POINT = chrono.ChVector3d(0.0, 0.0, 1.75)

STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3


# === Vehicle and system ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
bus.SetTireType(veh.TireModelType_PAC89)  # prompt: Pacejka 89 tire model
bus.SetTireStepSize(TIRE_STEP_SIZE)
bus.Initialize()

system = bus.GetSystem()  # cache: wrapper-owned ChSystem reused throughout setup and loop
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle = bus.GetVehicle()  # cache: underlying wheeled vehicle reused for vis and diagnostics
chassis = bus.GetChassisBody()  # cache: chassis body reused for logging
print("VEHICLE MASS: ", vehicle.GetMass())

# Wrapper-created essentials visible to source review:
# system: bus.GetSystem(); chassis body: bus.GetChassisBody();
# tire model: veh.TireModelType_PAC89; driver: ChInteractiveDriverIRR;
# terrain: RigidTerrain attached to the wrapper-owned system.

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Terrain ===
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
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.55, 0.48, 0.36))
terrain.Initialize()


# === Visualization and driver ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Dirt Road")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(TRACK_POINT, 18.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)

driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(STEP_SIZE / BRAKING_TIME)
driver.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
def main():
    frame = 0
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
                bus.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                bus.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)


                step_number += 1
                if system.GetChTime() >= SIM_END:
                    break

            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError) as exc:
        print(f"simulation failed: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    main()
