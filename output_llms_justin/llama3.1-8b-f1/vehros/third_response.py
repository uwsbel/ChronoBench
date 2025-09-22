import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math
import pychrono.sensor as sens

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
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()  # Initialize the driver system.

    # Create the ROS manager and register handlers for communication.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())  # Register the clock handler to synchronize ROS with the simulation.
    # Register the driver inputs handler for ROS topic '~/input/driver_inputs'.
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    # Register the vehicle state handler to publish vehicle state to ROS topic '~/output/hmmwv/state'.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.Initialize()  # Initialize the ROS manager.

    # Create a sensor manager for the vehicle
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())

    # Create a sensor to capture the scene
    offset_pose = ch.ChFramed(ch.ChVector3d(-5, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    cam = sens.ChCameraSensor(
        update_rate,  # scanning rate in Hz
        offset_pose,   # offset pose
        image_width,   # image width
        image_height,   # image height
        fov            # camera's horizontal field of view
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)
    cam.Initialize()
    sens_manager.AddSensor(cam)

    # Create a lidar to capture point clouds
    offset_pose = ch.ChFramed(ch.ChVector3d(-5, 0, 2), ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        update_rate,  # scanning rate in Hz
        offset_pose,   # offset pose
        horizontal_samples,   # number of horizontal samples
        vertical_samples,     # number of vertical channels
        horizontal_fov,        # horizontal field of view
        max_vert_angle,
        min_vert_angle,        # vertical field of view
        100.0,                # max lidar range
        0.01                  # noise
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(exposure_time)
    lidar.Initialize()
    sens_manager.AddSensor(lidar)

    # Register a handler for the lidar data
    # This handler will publish the lidar data to a ROS topic
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/hmmwv/lidar/scan"))

    # Create a visualization box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, False)
    box.SetPos(offset_pose.GetOrigin() + ch.ChVector3d(0, 0, 1.5))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(veh.GetDataFile("logo_pychrono_alpha.png"))
    hmmwv.GetSystem().Add(box)

    # Initialize the sensor manager
    sens_manager.Initialize()

    # Start the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the total duration of the simulation.
    # setup render related variables
    # Set the simulation time step
    time_step = 1e-3
    step_number = 0
    # Time interval between two render frames
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
        sens_manager.Advance(time_step)

        # Update the ROS manager to handle data publishing.
        if not ros_manager.Update(time, time_step):
            break  # Exit loop if ROS manager update fails.
        step_number += 1

if __name__ == "__main__":
    # Sensor parameters
    update_rate = 5  # scanning rate in Hz
    image_width = 1280
    image_height = 720
    fov = 1.408
    lag = 0
    exposure_time = 0

    # Lidar parameters
    horizontal_samples = 4500
    vertical_samples = 32
    horizontal_fov = 2 * ch.CH_PI  # horizontal field of view (assuming 90 degree horizontal field of view)
    max_vert_angle = ch.CH_PI / 12.
    min_vert_angle = -ch.CH_PI / 6.
    # noise model for the lidar (Gaussian noise on each point)
    # The standard deviation of the noise is a function of range (higher range = higher noise)
    # This is a simplified model and real lidar noise is more complex
    # http://www.valoreo.com/papers/2018/efficient_lidar_simulation/efficient_lidar_simulation.pdf
    # http://www.valoreo.com/papers/2015/realtime_lidar_simulation(realtime_lidar_simulation.pdf)
    # noise = 10^(-A * r + B)
    # We use the following values for A and B to match the noise plot in the first paper
    # http://www.valoreo.com/papers/2018/efficient_lidar_simulation/figure_9.png
    # A = 0.0825
    # B = 3.43
    noise = lambda r: 10**(-0.0825 * r + 3.43)

    # Simulation step size
    time_step = 1e-3

    main()