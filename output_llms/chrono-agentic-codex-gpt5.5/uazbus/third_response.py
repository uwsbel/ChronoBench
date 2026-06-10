"""UAZ bus rigid-terrain mobility test.

This PyChrono 9.0 NSC simulation drives a UAZ bus forward with a constant
throttle command on a flat rigid terrain patch. A fixed box obstacle is placed
in the lane to test the vehicle's mobility with the requested rigid tire model.
"""

import traceback

import pychrono.core as chrono
import pychrono.vehicle as veh


# === Constants ===
# Named values keep the requested vehicle, obstacle, and timing visible.
STEP_SIZE = 2.0e-3
TIRE_STEP_SIZE = 1.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH = 80.0
TERRAIN_WIDTH = 20.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

INIT_LOC = chrono.ChVector3d(0.0, 0.0, 0.5)
INIT_ROT = chrono.QUNIT
THROTTLE_VALUE = 0.5
STEERING_VALUE = 0.0
BRAKING_VALUE = 0.0

OBSTACLE_SIZE_X = 0.5
OBSTACLE_SIZE_Y = 5.0
OBSTACLE_SIZE_Z = 0.2
OBSTACLE_POS = chrono.ChVector3d(5.0, 0.0, 0.1)
OBSTACLE_DENSITY = 1000.0


# === Vehicle setup ===
# Configure the catalog UAZ bus as a wrapper-owned rigid-terrain vehicle.
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_RIGID)  # prompt: rigid tire model
vehicle.SetTireStepSize(TIRE_STEP_SIZE)
vehicle.Initialize()

system = vehicle.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

chassis = vehicle.GetChassisBody()  # cache: fetched once, reused for logging and camera chase
vehicle_core = vehicle.GetVehicle()  # cache: wrapper vehicle handle for driver and visualization

# Wrapper-created components are the vehicle-owned system, chassis/suspension
# bodies, wheel/tire subsystems, powertrain, driver, terrain, and Irrlicht view.


# === Terrain ===
# A rigid NSC terrain patch provides flat support for the vehicle and obstacle.
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
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 40, 10)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


# === Obstacle ===
# The requested fixed box obstacle sits ahead of the UAZ bus on the terrain.
obstacle_mat = chrono.ChContactMaterialNSC()
obstacle_mat.SetFriction(0.8)
obstacle_mat.SetRestitution(0.01)
obstacle = chrono.ChBodyEasyBox(
    OBSTACLE_SIZE_X,
    OBSTACLE_SIZE_Y,
    OBSTACLE_SIZE_Z,
    OBSTACLE_DENSITY,
    True,
    True,
    obstacle_mat,
)
obstacle.SetName("fixed_box_obstacle")
obstacle.SetPos(OBSTACLE_POS)
obstacle.SetFixed(True)
obstacle.EnableCollision(True)
system.AddBody(obstacle)


# === Visualization ===
# Vehicle Irrlicht rendering follows the catalog vehicle visual-system pattern.
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZ bus rigid tire obstacle test")
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 8.0, 2.0)
vis.AttachVehicle(vehicle_core)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AddGrid(
    2.0,
    2.0,
    40,
    10,
    chrono.ChCoordsysd(),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Driver ===
# A simple scored-core driver keeps throttle at the requested constant value.
driver = veh.ChDriver(vehicle_core)
driver.Initialize()
realtime_timer = chrono.ChRealtimeStepTimer()


# === Main loop ===
# Run the vehicle subsystem stack with constant scripted inputs.
def run_simulation():

    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

            for _ in range(RENDER_EVERY):
                time = system.GetChTime()
                driver.SetSteering(STEERING_VALUE)
                driver.SetThrottle(THROTTLE_VALUE)
                driver.SetBraking(BRAKING_VALUE)
                driver_inputs = driver.GetInputs()  # cache: synchronized once per step

                driver.Synchronize(time)
                terrain.Synchronize(time)
                vehicle.Synchronize(time, driver_inputs, terrain)
                vis.Synchronize(time, driver_inputs)


                driver.Advance(STEP_SIZE)
                terrain.Advance(STEP_SIZE)
                vehicle.Advance(STEP_SIZE)
                vis.Advance(STEP_SIZE)
                realtime_timer.Spin(STEP_SIZE)

                if system.GetChTime() >= SIM_END:
                    break
    except (RuntimeError, ValueError) as exc:  # solver divergence / invalid state guard
        traceback.print_exc()
        raise
    except (OSError, IOError) as exc:  # disk or recording output guard
        traceback.print_exc()
        raise


# === Entrypoint ===
# Keep construction and execution in one self-contained script.
if __name__ == "__main__":
    run_simulation()
