import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math
import pychrono.sensor as sens  # Added for sensor functionalities

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    
    # Create the HMMWV vehicle and set its parameters.
    hmmwv = veh.HMMWV()  # Corrected class name from HMMWV_Full to HMMWV (if applicable, based on current Chrono version)
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
    
    # Create visualization box
    system = hmmwv.GetSystem()
    box = ch.ChBodyEasyBox(2, 2, 0.5, 1000)  # Create a box
    box.SetPos(ch.ChVectorD(5, 0, 0.25))  # Position on terrain
    box.SetBodyFixed(True)
    system.Add(box)
    
    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    # Fixed terrain coordinate system parameter
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysD(), 100.0, 100.0)  
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()
    
    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(ch.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain with Lidar')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    # Updated camera position
    vis.AddCamera(ch.ChVectorD(-5, 2.5, 1.5), ch.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVectorD(1.5, -2.5, 5.5), ch.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)
    
    # Driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()
    
    # ROS setup
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    # Sensor setup
    sens_manager = sens.ChSensorManager(system)
    lidar = sens.ChLidarSensor()
    lidar.SetName("lidar")
    lidar.SetPosition(ch.ChVectorD(0, 0, 1.5))  # Position relative to chassis
    lidar.SetDirection(ch.ChVectorD(0, 1, 0))  # Forward direction
    lidar.SetFovHorizontal(math.radians(70))
    lidar.SetFovVertical(math.radians(30))
    lidar.SetMinRange(0.1)
    lidar.SetMaxRange(100)
    lidar.SetSamplingDistance(0.1)
    lidar.SetHorizontalResolution(360)
    lidar.SetVerticalResolution(10)
    lidar.SetUpdateRate(1.0 / 25)  # Match render FPS
    
    # Add filters (example)
    distance_filter = sens.ChFilterDistance()
    distance_filter.SetMinDistance(0.1)
    distance_filter.SetMaxDistance(100)
    lidar.AddFilter(distance_filter)
    
    sens_manager.AddSensor(lidar)
    
    # ROS Lidar handler
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar_data"))
    ros_manager.Initialize()
    
    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)
    step_number = 0
    
    while vis.Run():
        time = system.GetChTime()
        
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