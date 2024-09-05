import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Added import for sensor functionalities
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVectorD(0, 0, 1.6), ch.ChQuaternionD(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.ChVectorD(0, 0, 0), ch.ChVectorD(0, 0, 1), 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Added visualization box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True)
    box.SetPos(ch.ChVectorD(5, 0, 0.5))
    hmmwv.GetSystem().Add(box)

    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVectorD(-5, 2.5, 1.5), ch.ChVectorD(0, 0, 1))  # Changed camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVectorD(1.5, -2.5, 5.5), ch.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Set up ChSensorManager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    sens_manager.scene.AddPointLight(ch.ChVectorF(0, 0, 100), ch.ChVectorF(1, 1, 1), 500)

    # Add and configure ChLidarSensor
    offset_pose = ch.ChFrameD(ch.ChVectorD(0, 0, 1.5), ch.ChQuaternionD(1, 0, 0, 0))
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),          # body lidar is attached to
        25,                              # scanning rate in Hz
        offset_pose,                     # offset pose
        1000,                            # number of horizontal samples
        16,                              # number of vertical channels
        ch.CH_C_PI,                      # horizontal field of view
        ch.CH_C_PI / 12,                 # vertical field of view
        100                              # max distance
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.0)

    # Add various lidar filters
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterXYZI())
    lidar.PushFilter(sens.ChFilterLidarIntensityClip())
    lidar.PushFilter(sens.ChFilterLidarNoise(0.01))
    lidar.PushFilter(sens.ChFilterVis())

    sens_manager.AddSensor(lidar)

    # Create the ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    # Register ChROSLidarHandler to publish lidar data
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar", 25))
    
    ros_manager.Initialize()

    # Simulation loop
    time_step = 1e-3
    step_number = 0
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update sensor manager
        sens_manager.Update()

        if not ros_manager.Update(time, time_step):
            break

        step_number += 1

if __name__ == "__main__":
    main()