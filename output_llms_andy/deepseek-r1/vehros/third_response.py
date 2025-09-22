import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens
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
    
    # Add visualization box
    box_body = ch.ChBodyEasyBox(2, 2, 1,  # Dimensions (length, width, height)
                                1000,     # Density (kg/m³)
                                True,      # Enable visualization
                                True)      # Enable collision
    box_body.SetPos(ch.ChVector3d(5, 0, 0.5))  # Position away from vehicle
    box_body.SetFixed(True)  # Make it stationary
    hmmwv.GetSystem().Add(box_body)
    
    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  # Updated camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()  # Initialize the driver system.

    # Create sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    sens_manager.scene.AddPointLight(ch.ChVector3d(0, 0, 100), ch.ChColor(1, 1, 1), 500)

    # Configure lidar sensor
    lidar_offset_pose = ch.ChFramed(ch.ChVector3d(0, 0, 1.5),  # Position on chassis
                                   ch.QuatFromAngleY(0))       # Orientation
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  # Parent body
        10,                      # Update rate (Hz)
        lidar_offset_pose,       # Offset pose
        900,                     # Horizontal samples
        30,                      # Vertical samples
        2 * ch.CH_PI,            # Horizontal FOV (360°)
        ch.CH_PI / 6,            # Vertical FOV (30°)
        100.0,                   # Max detection distance
        sens.LidarModel_Raycast,
        sens.LidarBeamShape_Rectangular)
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0.1)
    lidar.SetCollectionWindow(0.02)

    # Add lidar filters
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZ(0.01, 0.002))  # Noise model
    lidar.PushFilter(sens.ChFilterLidarIntensityThreshold(0.1, 1.0))  # Intensity filter
    lidar.PushFilter(sens.ChFilterPCfromDepth())  # Point cloud from depth
    sens_manager.AddSensor(lidar)

    # Create the ROS manager and register handlers
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    # Register lidar handler
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(lidar, "~/output/lidar"))
    ros_manager.Initialize()

    # Start the simulation loop.
    time = 0
    time_step = 1e-3  # Define the simulation time step.
    time_end = 30  # Set the total duration of the simulation.
    step_number = 0
    # Time interval between two render frames
    render_step_size = 1.0 / 25  # FPS = 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)  # Enable real-time simulation for the vehicle.
    
    while vis.Run():  # Run the simulation loop.
        time = hmmwv.GetSystem().GetChTime()  # Update simulation time.
        
        # Render the scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Get driver inputs and synchronize systems
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)

        # Advance simulation
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)

        # Update sensor manager
        sens_manager.Update()

        # Update ROS manager
        if not ros_manager.Update(time, time_step):
            break
            
        step_number += 1

if __name__ == "__main__":
    main()