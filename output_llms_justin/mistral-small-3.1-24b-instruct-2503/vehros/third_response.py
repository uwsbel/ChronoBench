import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Added to use sensor functionalities
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  # Set the contact method for physics.
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # Disable collision for the chassis.
    hmmwv.SetChassisFixed(False)  # Make the chassis movable.
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))  # Initialize vehicle position and orientation.
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  # Use shaft-based engine model.
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # Use automatic transmission with shafts.
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)  # Set all-wheel drive.
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)  # Use pitman arm steering.
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Set tire model.
    hmmwv.SetTireStepSize(1e-3)  # Set the tire simulation step size.
    hmmwv.Initialize()  # Initialize the vehicle.
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the terrain for the vehicle to interact with.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()  # Create a contact material for the terrain.
    patch_mat.SetFriction(0.9)  # Set friction for the terrain.
    patch_mat.SetRestitution(0.01)  # Set restitution (bounciness) for the terrain.
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)  # Add a patch to the terrain.
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()  # Initialize the terrain.

    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  # Updated camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()  # Initialize the driver system.

    # Create the ROS manager and register handlers for communication.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Register the clock handler to synchronize ROS with the simulation.
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    # Create a visualization box using ChBodyEasyBox
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    box.SetPos(ch.ChVector3d(5, 0, 0.5))
    box.SetBodyFixed(True)
    hmmwv.GetSystem().Add(box)

    # Set up ChSensorManager to manage sensors
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    hmmwv.GetSystem().Add(sens_manager)

    # Add and configure a ChLidarSensor with various filters
    lidar_sensor = sens.ChLidarSensor(hmmwv.GetVehicle())
    lidar_sensor.SetName("lidar_sensor")
    lidar_sensor.SetPos(ch.ChVector3d(0, 0, 2))
    lidar_sensor.SetRot(ch.ChMatrix33d(ch.ChVector3d(0, 0, 1), ch.ChVector3d(0, 1, 0)))
    lidar_sensor.SetScanRate(10)
    lidar_sensor.SetRangeMax(100)
    lidar_sensor.SetRangeMin(0.1)
    lidar_sensor.SetRangeNoise(0.01)
    lidar_sensor.SetRangeBias(0)
    lidar_sensor.SetAngularResolution(1)
    lidar_sensor.SetVerticalResolution(1)
    lidar_sensor.SetVerticalFOV(-10, 10)
    lidar_sensor.SetHorizontalFOV(-90, 90)
    lidar_sensor.SetFilterRangeMin(0.5)
    lidar_sensor.SetFilterRangeMax(50)
    lidar_sensor.SetFilterNoise(0.005)
    lidar_sensor.SetFilterBias(0.001)
    lidar_sensor.SetFilterGround(1)
    lidar_sensor.SetFilterGroundNoise(0.005)
    lidar_sensor.SetFilterGroundBias(0.001)
    lidar_sensor.SetFilterDynamic(1)
    lidar_sensor.SetFilterDynamicNoise(0.005)
    lidar_sensor.SetFilterDynamicBias(0.001)
    lidar_sensor.SetFilterStatic(1)
    lidar_sensor.SetFilterStaticNoise(0.005)
    lidar_sensor.SetFilterStaticBias(0.001)
    lidar_sensor.SetFilterAngleMin(-10)
    lidar_sensor.SetFilterAngleMax(10)
    sens_manager.AddSensor(lidar_sensor)

    # Register ChROSLidarHandler to publish lidar data to ROS
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar_sensor, "~/output/lidar"))

    ros_manager.Initialize()  # Initialize the ROS manager.

    # Start the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the total duration of the simulation.
    step_number = 0
    render_step_size = 1.0 / 25  # FPS = 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)  # Enable real-time simulation for the vehicle.

    while vis.Run():  # Run the simulation loop.
        time = hmmwv.GetSystem().GetChTime()  # Update simulation time.
        # Render the scene
        if (step_number % render_steps == 0):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        # Get driver inputs and synchronize the vehicle, terrain, and driver systems.
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)  # Synchronize the driver system.
        terrain.Synchronize(time)  # Synchronize the terrain.
        hmmwv.Synchronize(time, driver_inputs, terrain)  # Synchronize the vehicle with inputs and terrain.

        # Advance the simulation for all modules by one timestep.
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update the ROS manager to handle data publishing.
        if not ros_manager.Update(time, time_step):
            break  # Exit loop if ROS manager update fails.

        # Update the sensor manager
        sens_manager.Update()

        step_number += 1

if __name__ == "__main__":
    main()