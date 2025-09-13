import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Added sensor module
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
    
    # Create visualization box (Instruction 2)
    box_body = ch.ChBodyEasyBox(1.0, 1.0, 1.0, 1000, True, True)
    box_body.SetPos(ch.ChVector3d(5, 0, 0.5))
    box_body.SetFixed(True)
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
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  # Modified camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    # Create sensor manager (Instruction 3)
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    
    # Create lidar sensor (Instruction 4)
    lidar_offset_pose = ch.ChFramed(ch.ChVector3d(0, 0, 2.0), ch.Q_from_AngZ(0))
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),   # Parent body
        10,                       # Update rate (Hz)
        lidar_offset_pose,        # Offset pose
        900,                      # Horizontal samples
        30,                       # Vertical channels
        math.radians(360),        # Horizontal FOV
        math.radians(30),         # Max vertical angle
        math.radians(-30),        # Min vertical angle
        100.0                     # Range
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(0.1)
    lidar.SetCollectionWindow(0.02)
    
    # Add lidar filters
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualize(900, 30, "Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data/"))
    sens_manager.AddSensor(lidar)
    
    # Register ROS lidar handler (Instruction 5)
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))
    
    # Initialize ROS manager after all handlers are registered
    ros_manager.Initialize()

    # Simulation settings
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25  # FPS = 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)
    
    # Simulation loop
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        
        # Render scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver_inputs.m_steering = 0  # Example steering input
        driver_inputs.m_throttle = 0.5  # Example throttle input
        
        # Synchronize systems
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        # Advance systems
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        
        # Update sensors (Instruction 6)
        sens_manager.Update()
        
        # Update ROS
        if not ros_manager.Update(time, time_step):
            break
        
        step_number += 1

if __name__ == "__main__":
    main()