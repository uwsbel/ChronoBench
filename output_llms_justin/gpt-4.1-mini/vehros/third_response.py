import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
from pychrono import irrlicht as chronoirr
import math


def main():
    # Set the data path for vehicle files
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')

    # Create the HMMWV vehicle and set its parameters
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  # Set the contact method for physics
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # Disable collision for the chassis
    hmmwv.SetChassisFixed(False)  # Make the chassis movable
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))  # Initialize vehicle position and orientation
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  # Use shaft-based engine model
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # Use automatic transmission with shafts
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)  # Set all-wheel drive
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)  # Use pitman arm steering
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Set tire model
    hmmwv.SetTireStepSize(1e-3)  # Set the tire simulation step size
    hmmwv.Initialize()  # Initialize the vehicle
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create a visualization box using ChBodyEasyBox (size 0.5x0.5x0.5, 1000 kg, collision enabled, visualization enabled)
    box = ch.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, True)
    box.SetPos(ch.ChVectorD(3, 0, 0.25))  # Place the box beside the vehicle
    box.SetBodyFixed(False)
    hmmwv.GetSystem().Add(box)

    # Create the terrain for the vehicle to interact with
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChMaterialSurfaceNSC()  # Use NSC contact material instead of deprecated ChContactMaterialNSC
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    # Changed camera position for a new perspective as per instructions
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create the ROS manager and register handlers for communication
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    # Set up the sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    sens_manager.SetVerbose(True)

    # Create and configure a Lidar sensor attached to the chassis
    lidar_mount_pose = ch.ChFrameD(ch.ChVectorD(0.5, 0, 1.7))  # Slightly above and in front of chassis center
    lidar_sensor = sens.ChLidarSensor(hmmwv.GetChassisBody(), 0, lidar_mount_pose, 16000)
    lidar_sensor.SetFOV(120)
    lidar_sensor.SetHorizontalResolution(1024)
    lidar_sensor.SetChannels(1)
    lidar_sensor.SetMaxDistance(50)
    lidar_sensor.SetVerticalFOV(20)
    lidar_sensor.SetVerticalResolution(64)
    # Add filters to lidar sensor
    lidar_sensor.FilterList().push_back(sens.ChFilterDoubleEcho())
    lidar_sensor.FilterList().push_back(sens.ChFilterClippingDist(0.5, 30))
    lidar_sensor.FilterList().push_back(sens.ChFilterDropPulse(0.07))
    lidar_sensor.FilterList().push_back(sens.ChFilterCloudOpen(0.15, 12))
    sens_manager.AddSensor(lidar_sensor)

    # Register ROS lidar handler to publish lidar points to ROS topic
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar_sensor, "~/output/lidar_points"))

    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25  # 25 FPS
    render_steps = math.ceil(render_step_size / time_step)

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run() and hmmwv.GetSystem().GetChTime() < time_end:
        time = hmmwv.GetSystem().GetChTime()

        # Render the scene at a fixed rate
        if (step_number % render_steps) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Get driver inputs and synchronize the vehicle, terrain, driver systems
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # Advance simulation by one timestep
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update sensor manager (important to generate sensor data)
        sens_manager.Update()

        # Update ROS manager and exit if it fails
        if not ros_manager.Update(time, time_step):
            break

        step_number += 1


if __name__ == "__main__":
    main()