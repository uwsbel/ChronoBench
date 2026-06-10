"""HMMWV vehicle simulation on rigid terrain with ROS communication.

This PyChrono 9.0.0 script builds an NSC HMMWV_Full vehicle with shafts
powertrain components and TMeasy tires, drives it over a flat rigid terrain,
and registers ROS handlers for simulation clock, driver inputs, and chassis
state. The vehicle, terrain, driver, visualization, and ROS manager are
synchronized and advanced each timestep.
"""

import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.ros as chros


# === Constants ===
# Named simulation and vehicle parameters keep the setup auditable and reusable.
STEP_SIZE = 1.0e-3
TIRE_STEP_SIZE = STEP_SIZE
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 100.0
TERRAIN_WIDTH = 100.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.6)
INIT_ROT = chrono.QUNIT
STEERING_TIME = 1.0
THROTTLE_TIME = 1.0
BRAKING_TIME = 0.3
ROS_PUBLISH_RATE = 25.0


# === Vehicle and terrain ===
# The HMMWV wrapper owns the Chrono system used by terrain, driver, and ROS.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
hmmwv.SetTireType(veh.TireModelType_TMEASY)  # prompt: explicit tire model
hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
hmmwv.Initialize()

system = hmmwv.GetSystem()  # cache: wrapper-owned system reused by all subsystems
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

vehicle = hmmwv.GetVehicle()  # cache: vehicle interface reused by driver and diagnostics
chassis = hmmwv.GetChassisBody()  # cache: chassis body reused by ROS and logging
chassis.SetName("hmmwv_chassis")

# Wrapper-created core components: vehicle system, chassis body, suspension and
# wheel assemblies, powertrain, tire subsystems, terrain, driver, visualization,
# and ROS handlers all share the single wrapper-owned system above.

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.QUNIT),
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Visualization ===
# Vehicle Irrlicht visualization provides the interactive runtime view.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV ROS vehicle simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle)


# === Driver and ROS ===
# The driver receives ROS input messages while default deltas keep keyboard control valid.
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(STEP_SIZE / STEERING_TIME)
driver.SetThrottleDelta(STEP_SIZE / THROTTLE_TIME)
driver.SetBrakingDelta(STEP_SIZE / BRAKING_TIME)
driver.Initialize()

ros_manager = chros.ChROSPythonManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())
ros_manager.RegisterHandler(
    chros.ChROSDriverInputsHandler(ROS_PUBLISH_RATE, driver, "~/input/driver_inputs")
)
ros_manager.RegisterHandler(
    chros.ChROSBodyHandler(ROS_PUBLISH_RATE, chassis, "~/output/hmmwv_chassis")
)
ros_manager.Initialize()

realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
# Each frame renders once, then advances the full vehicle stack at fixed substeps.
def run_simulation():

    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()

            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                if time >= SIM_END:
                    break

                driver_inputs = driver.GetInputs()  # cache: one input fetch per substep

                driver.Synchronize(time)
                terrain.Synchronize(time)
                hmmwv.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)

                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                hmmwv.Advance(STEP_SIZE)

                if not ros_manager.Update(time, STEP_SIZE):
                    return

                realtime_timer.Spin(STEP_SIZE)


    except (RuntimeError, ValueError, OSError) as exc:
        traceback.print_exc()
        raise
    finally:
        pass

    return


if __name__ == "__main__":
    rows = run_simulation()
