#
#  HMMWV demo (PyChrono)
#  ----------------------
#  - rigid‐terrain vehicle simulation
#  - real–time Irrlicht visualization
#  - ROS interface (vehicle state, driver inputs, lidar data)
#  - Chrono::Sensor LiDAR (with a couple of illustrative filters)
#

import math
import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens                    # ❶ sensor module
from   pychrono import irrlicht as chronoirr


def main():

    # ----------------------------------------------------------
    # Chrono / data folders
    # ----------------------------------------------------------
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    sens.SetChronoDataPath(ch.GetChronoDataPath() + 'sensor/')

    # ----------------------------------------------------------
    # 1.  Vehicle (HMMWV full model)
    # ----------------------------------------------------------
    hmmwv = veh.HMMWV_Full()

    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)

    hmmwv.SetInitPosition(
        ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6),   # x-y-z
                       ch.ChQuaterniond(1, 0, 0, 0))
    )

    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    # visual meshes
    hmmwv.SetChassisVisualizationType    (veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType (veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType   (veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType      (veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType       (veh.VisualizationType_MESH)

    system = hmmwv.GetSystem()

    # ----------------------------------------------------------
    # 2.  Terrain
    # ----------------------------------------------------------
    terrain          = veh.RigidTerrain(system)
    patch_material   = ch.ChContactMaterialNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)

    patch = terrain.AddPatch(patch_material, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # ----------------------------------------------------------
    # 3.  A simple visual box (just to have one more object)
    # ----------------------------------------------------------
    box_body = ch.ChBodyEasyBox(
        1.0, 1.0, 1.0,          # size [m]
        1000,                   # density [kg/m^3]
        True,                   # collide?
        True,                   # visualise?
        patch_material)
    box_body.SetPos(ch.ChVector3d(2, 0, 2))
    system.Add(box_body)

    # ----------------------------------------------------------
    # 4.  Chrono::Sensor – manager and LiDAR
    # ----------------------------------------------------------
    sens_manager = sens.ChSensorManager(system)

    # environment lights for ray-based sensors
    sens_manager.scene.AddPointLight(ch.ChVector3f(0, 0, 10), ch.ChColor(1, 1, 1), 800)

    lidar_offset = ch.ChFramed(ch.ChVector3d( 0.8, 0, 1.5),   # position wrt chassis
                               ch.ChQuaterniond(1, 0, 0, 0))  # orientation

    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),     # parent body
        10.0,                       # update rate [Hz]
        lidar_offset,               # offset pose
        640,                        # horizontal samples
        32,                         # vertical channels
        math.radians(30.0),         # vertical FOV  (total)
        math.radians(360.0),        # horizontal FOV
        100.0)                      # maximum range [m]

    lidar.SetName("HMMWV_LiDAR")

    # illustrative filter chain
    lidar.PushFilter(sens.ChFilterLidarNoise(0.02))
    lidar.PushFilter(sens.ChFilterLidarXYZReturn())   # create XYZ point cloud
    lidar.PushFilter(sens.ChFilterAccess())           # allow CPU access

    sens_manager.AddSensor(lidar)

    # ----------------------------------------------------------
    # 5.  Irrlicht visualisation
    # ----------------------------------------------------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV – rigid terrain + LiDAR')
    vis.Initialize()

    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()

    # changed camera position (instruction 7)
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))

    vis.AddTypicalLights()
    vis.AddLightWithShadow(
        ch.ChVector3d(1.5, -2.5, 5.5),   # light position
        ch.ChVector3d(0, 0, 0.5),        # target
        3, 4, 10, 40, 512)

    # ----------------------------------------------------------
    # 6.  Driver (blank manual driver – keeps inputs at zero)
    # ----------------------------------------------------------
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # ----------------------------------------------------------
    # 7.  ROS interface
    # ----------------------------------------------------------
    ros_manager = chros.ChROSPythonManager()

    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(
        chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(
        chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(
        chros.ChROSLidarHandler(10, lidar, "~/output/hmmwv/lidar"))

    ros_manager.Initialize()

    # ----------------------------------------------------------
    # 8.  Simulation loop
    # ----------------------------------------------------------
    time_step          = 1e-3
    render_fps         = 25
    render_step_size   = 1.0 / render_fps
    render_steps       = math.ceil(render_step_size / time_step)
    step_number        = 0

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():

        time = system.GetChTime()

        # ---------------------------------- rendering
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # ---------------------------------- module synchronisation
        inputs = driver.GetInputs()

        driver.Synchronize (time)
        terrain.Synchronize(time)
        hmmwv.Synchronize  (time, inputs, terrain)

        # ---------------------------------- advance dynamics
        driver.Advance (time_step)
        terrain.Advance(time_step)
        hmmwv.Advance  (time_step)

        # ---------------------------------- sensors
        sens_manager.Update()                       # ❻ update sensor manager

        # ---------------------------------- ROS
        if not ros_manager.Update(time, time_step):
            break                                   # terminate if ROS node asked so

        step_number += 1


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    main()