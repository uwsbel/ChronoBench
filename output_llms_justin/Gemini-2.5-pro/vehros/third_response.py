import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math
import pychrono.sensor as sens # Instruction 1: Added import

def main():
    # Set the path to Chrono data files (especially for vehicle models)
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')

    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)  # Use Non-Smooth Contact method.
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)  # Disable collision for the chassis itself.
    hmmwv.SetChassisFixed(False)  # Make the chassis movable.
    # Initialize vehicle position (at origin, slightly above ground) and orientation (identity quaternion).
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  # Use shaft-based engine model.
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  # Use automatic transmission.
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)  # Set all-wheel drive.
    hmmwv.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)  # Use pitman arm steering.
    hmmwv.SetTireType(veh.TireModelType_TMEASY)  # Set TMEasy tire model.
    hmmwv.SetTireStepSize(1e-3)  # Set the tire simulation step size.
    hmmwv.Initialize()  # Initialize the vehicle system.

    # Set visualization types for different parts of the vehicle.
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the terrain for the vehicle to interact with.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()  # Create a contact material for the terrain.
    patch_mat.SetFriction(0.9)  # Set friction coefficient.
    patch_mat.SetRestitution(0.01)  # Set restitution coefficient.
    # Add a large flat patch of terrain. Centered at origin. Increased size for Lidar.
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysd(ch.ChVector3d(0,0,0), ch.QUNIT), 200.0, 200.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) # Set texture for the terrain patch.
    terrain.Initialize()  # Initialize the terrain.

    # Instruction 2: Add a visualization box
    # Create a textured visualization box
    box_size = ch.ChVector3d(1.0, 1.0, 1.0)  # Dimensions of the box (length, width, height)
    # Position the box slightly above ground to avoid Z-fighting
    box_pos = ch.ChVector3d(5, 2, box_size.z / 2 + 1e-3)
    
    box_body = ch.ChBody() # Create a standard Chrono body for the box
    box_body.SetPos(box_pos)
    box_body.SetFixed(True) # Make the box static

    # Add collision shape to the box (using half-lengths)
    box_coll_shape = ch.ChCollisionShapeBox(patch_mat, box_size.x / 2, box_size.y / 2, box_size.z / 2)
    box_body.AddCollisionShape(box_coll_shape)
    box_body.EnableCollision(True) # Enable collision for the box

    # Add visual shape to the box (using full lengths)
    box_visual_shape = ch.ChVisualShapeBox(box_size.x, box_size.y, box_size.z)
    try:
        # Attempt to load and set a texture for the box
        box_visual_shape.SetTexture(ch.GetChronoDataFile("textures/concrete.jpg"))
    except Exception as e:
        print(f"Warning: Could not set texture for visualization box: {e}. Using default color.")
    box_body.AddVisualShape(box_visual_shape)
    
    hmmwv.GetSystem().Add(box_body) # Add the box body to the physics system

    # Create run-time visualization using Irrlicht.
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z) # Set Z as the vertical axis for the camera.
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV with Lidar, ROS, and Visualization Box') # Updated window title.
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    # Instruction 7: Changed camera position and target point for a better view.
    vis.AddCamera(ch.ChVector3d(-8, 4, 2.5), ch.ChVector3d(0, 0, 0.5))
    vis.AddTypicalLights()
    # Add a light source with shadow casting capabilities.
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Instruction 3: Set up ChSensorManager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    # Set raycasting method. RendersharpEngine.IRRSHADER uses Irrlicht's Z-buffer (good if Irrlicht is active).
    # Fallback to CPU-based method if GPU/Rendersharp setup fails.
    try:
        sens_manager.SetRaycastingMethod(sens.RendersharpEngine.IRRSHADER, 4) # Number of sample per pixel/ray
    except Exception as e:
        print(f"Warning: Could not set Rendersharp IRRSHADER. Error: {e}. Falling back to CPU raycasting.")
        sens_manager.SetRaycastingMethod(sens.RaycastingMethod_RAYSHAFT_SW, 4)
    sens_manager.SetVerbose(False) # Disable verbose output from sensor manager.

    # Instruction 4: Add and configure a ChLidarSensor with various filters
    lidar_update_rate = 10  # Hz (Lidar scan frequency)
    # Offset pose of the Lidar sensor relative to the HMMWV chassis (e.g., on the roof).
    lidar_offset_pose = ch.ChFrameD(ch.ChVector3d(0.0, 0, 0.8), ch.QUNIT) 
    
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),      # Body to which the Lidar is attached.
        lidar_update_rate,           # Update rate in Hz.
        lidar_offset_pose,           # Offset pose relative to the parent body.
        2000,                        # Number of horizontal samples (azimuthal resolution).
        32,                          # Number of vertical channels/samples (polar resolution).
        2 * math.pi,                 # Horizontal field of view (360 degrees).
        math.radians(15),            # Maximum vertical angle (e.g., +15 degrees from sensor's XZ plane).
        math.radians(-15),           # Minimum vertical angle (e.g., -15 degrees from sensor's XZ plane).
        100.0,                       # Maximum detection range in meters.
        sens.LidarBeamShape_RECTANGULAR, # Shape of the Lidar beam.
        1,                           # Sample radius for beam shape (pixels).
        0.003,                       # Divergence angle for the beam.
        0.1                          # Near clipping distance (ignores points closer than this).
    )
    lidar.SetName("LidarSensor")
    lidar.SetLag(1 / lidar_update_rate) # Simulate processing delay.
    lidar.SetMaxCollectionWindow(0.0)   # Data collection window relative to lag.

    # Add filters to process Lidar data.
    lidar.PushFilter(sens.ChFilterPCfromDepth())      # Generates point cloud from depth map.
    lidar.PushFilter(sens.ChFilterIntensityFromRange()) # Calculates point intensity based on range.
    lidar.PushFilter(sens.ChFilterXYZIAccess())       # Provides access to point data (X, Y, Z, Intensity).
    # Optional: Add noise filter for more realistic simulation
    # noise_model = sens.ChNoiseNormal(ch.ChVector3d(0,0,0), ch.ChVector3d(0.01, 0.01, 0.01)) # Example noise
    # lidar.PushFilter(sens.ChFilterDistortionPoints(noise_model))

    sens_manager.AddSensor(lidar) # Add Lidar to the sensor manager.

    # Create the ROS manager and register handlers for communication.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler()) # Publishes simulation time to /clock.
    # Subscribes to driver inputs from ROS topic.
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    # Publishes vehicle chassis state (pose, velocity) to ROS topic.
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    # Instruction 5: Register ChROSLidarHandler to publish Lidar data to ROS.
    lidar_ros_frame_id = "lidar_link" # TF frame ID for the Lidar data in ROS.
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(
        lidar_update_rate,  # Publishing update rate (Hz), should match or be sensible with sensor rate.
        lidar,              # The Lidar sensor object.
        "~/output/hmmwv/lidar/point_cloud",  # ROS topic name for Lidar point cloud.
        lidar_ros_frame_id  # TF frame ID.
    ))
    
    try:
        ros_manager.Initialize()  # Initialize the ROS manager.
    except Exception as e:
        print(f"Error: Could not initialize ROS manager: {e}")
        print("Please ensure ROS master (roscore) is running and
print("error happened with only start ```python")