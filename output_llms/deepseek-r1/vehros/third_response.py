import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Added sensor module import
from pychrono import irrlicht as chronoirr
import math

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(ch.ChContactMethod_NSC)
    hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(ch.ChCoordsysd(ch.ChVector3d(0, 0, 1.6), ch.ChQuaterniond(1, 0, 0, 0)))
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
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()
    
    # Create visualization box (ChBodyEasyBox)
    box_body = ch.ChBodyEasyBox(2, 2, 1,  # Dimensions (x,y,z)
                                1000,     # Density (kg/m³)
                                True,     # Enable visualization
                                True)     # Enable collision
    box_body.SetPos(ch.ChVector3d(5, 0, 0.5))  # Position in world coordinates
    box_body.SetFixed(True)  # Make the box static
    box_body.GetVisualShape(0).SetColor(ch.ChColor(0.8, 0.2, 0.2))  # Red color
    hmmwv.GetSystem().Add(box_body)

    # Create run-time visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('HMMWV Vehicle Simulation with Sensors')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  # Updated camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create and initialize driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

    # Create sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    # Add ambient light for sensors
    sens_manager.scene.AddPointLight(ch.ChVector3d(0, 0, 100), ch.ChColor(1, 1, 1), 1000)

    # Create and configure lidar sensor
    lidar_offset = ch.ChFramed(ch.ChVector3d(0.5, 0, 0.5),  # Position offset (front of chassis)
                             ch.ChQuaterniond(1, 0, 0, 0))  # Rotation (identity)
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),  # Parent body
        30,                      # Update rate (Hz)
        lidar_offset,            # Offset pose
        1800,                    # Horizontal samples
        16,                      # Vertical samples
        ch.CH_PI,                # Horizontal FOV (180°)
        ch.CH_PI / 6,            # Vertical FOV (30°)
        100.0                    # Max detection distance
    )
    lidar.SetName("Vehicle Lidar")
    lidar.SetLag(0.0)
    lidar.SetCollectionWindow(0.02)

    # Add lidar filters
    lidar.PushFilter(sens.ChFilterPCfromDepth())  # Create point cloud from depth
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))  # Add noise
    lidar.PushFilter(sens.ChFilterVisualize(640, 360, "Lidar Data"))  # Visualization filter
    lidar.PushFilter(sens.ChFilterXYZIAccess())  # Allow data access
    sens_manager.AddSensor(lidar)

    # Register lidar handler with ROS
    ros_manager.RegisterHandler(sens.ChROSLidarHandler(lidar, "~/output/lidar/pointcloud"))
    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25  # FPS = 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        
        # Render scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Synchronize systems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        # Advance systems
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        
        # Update sensor manager (processes sensor data)
        sens_manager.Update()
        
        # Update ROS manager
        if not ros_manager.Update(time, time_step):
            break
        
        step_number += 1

if __name__ == "__main__":
    main()