"""HMMWV vehicle ROS bridge with rigid terrain and Irrlicht visualization.

This standalone PyChrono 9.0.0 script builds an NSC HMMWV on a textured rigid
terrain patch, publishes vehicle state and driver inputs through ChROS, and
renders the moving vehicle in an Irrlicht window. The vehicle follows a simple
scripted throttle schedule so chassis motion, terrain contact, ROS publishing,
and visual meshes are visible during a short validation run.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.ros as chros
import pychrono.vehicle as veh


# === Constants ===
# Direct constants keep the vehicle demo close to the catalog examples.
STEP_SIZE = 2.0e-3
SIM_END = 8.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
HMMWV_INIT_POS = chrono.ChVector3d(0.0, 0.0, 0.6)
HMMWV_INIT_ROT = chrono.QUNIT


class CruiseDriver(veh.ChDriver):
    """Scripted driver used by the ROS input handler and the vehicle loop."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        if time < 0.5:
            self.SetThrottle(0.0)
            self.SetBraking(0.2)
        else:
            self.SetThrottle(0.45)
            self.SetBraking(0.0)
        self.SetSteering(0.15 * math.sin(0.45 * time))


# === Vehicle ===
# The wrapper owns its Chrono system; terrain, ROS, and visualization use it.
def build_vehicle():
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(HMMWV_INIT_POS, HMMWV_INIT_ROT))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(STEP_SIZE)
    hmmwv.Initialize()

    system = hmmwv.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    chassis = hmmwv.GetChassisBody()  # cache: reused for ROS, validation, and logs
    chassis.SetName("hmmwv_chassis")
    return hmmwv, system, chassis


# === Terrain ===
# A rigid textured terrain patch supplies stable tire contact.
def build_terrain(system):
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
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    return terrain


# === ROS bridge ===
# Clock, chassis state, TF, and driver-input topics expose the running vehicle.
def build_ros(system, chassis, driver):
    base_link = chrono.ChBody()
    base_link.SetName("base_link")
    base_link.SetFixed(True)
    system.AddBody(base_link)

    ros_manager = chros.ChROSPythonManager("chrono_vehros")
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25.0, chassis, "~/hmmwv/chassis"))
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25.0, driver, "~/hmmwv/driver_inputs"))
    tf_handler = chros.ChROSTFHandler(25.0)
    tf_handler.AddTransform(base_link, base_link.GetName(), chassis, chassis.GetName())
    ros_manager.RegisterHandler(tf_handler)
    ros_manager.Initialize()
    return ros_manager, base_link


# === Visualization ===
# The prompt requires a runtime Irrlicht visualization block for the vehicle.
def build_visualization(system):
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("HMMWV ROS Irrlicht Visualization")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(-8, -10, 4), chrono.ChVector3d(12, 0, 1.0))
    vis.AddTypicalLights()
    vis.AddGrid(
        5.0,
        5.0,
        40,
        40,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.02), chrono.QUNIT),
        chrono.ChColor(0.35, 0.35, 0.35),
    )
    return vis


# === Main loop ===
# Synchronize and advance driver, terrain, vehicle, visualization, and ROS.
def run_simulation():
    hmmwv, system, chassis = build_vehicle()
    terrain = build_terrain(system)
    driver = CruiseDriver(hmmwv.GetVehicle())
    driver.Initialize()
    ros_manager, base_link = build_ros(system, chassis, driver)
    vis = build_visualization(system)
    realtime_timer = chrono.ChRealtimeStepTimer()


    step_number = 0
    try:
        while vis.Run() and system.GetChTime() < SIM_END:
            time = system.GetChTime()
            if step_number % RENDER_EVERY == 0:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()

            driver.Synchronize(time)
            driver_inputs = driver.GetInputs()
            terrain.Synchronize(time)
            hmmwv.Synchronize(time, driver_inputs, terrain)

            driver.Advance(STEP_SIZE)
            terrain.Advance(STEP_SIZE)
            hmmwv.Advance(STEP_SIZE)

            speed = chassis.GetPosDt().Length()  # cache: one velocity magnitude per step
            pos = chassis.GetPos()  # cache: one chassis position read per step

            if not ros_manager.Update(system.GetChTime(), STEP_SIZE):
                break

            step_number += 1
            realtime_timer.Spin(STEP_SIZE)
    except (RuntimeError, ValueError, OSError) as exc:
        print(f"simulation failed during vehicle/ROS execution: {exc}")
        raise
    finally:
        pass


if __name__ == "__main__":
    run_simulation()
