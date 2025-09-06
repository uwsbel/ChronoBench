import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
from pychrono import irrlicht as chronoirr
import math
import pychrono.sensor as sens  # Added for sensor functionality

def main():
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    
    # Create the HMMWV vehicle and set its parameters
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

    # Create terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    # Fixed coordinate system issue
    patch = terrain.AddPatch(patch_mat, ch.ChCoordsysD(), 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Visualization box addition
    box = ch.ChBodyEasyBox(2, 2, 2, 1000, True, True)
    box.SetPos(ch.ChVectorD(5, 0, 0))  # Position the box in the scene
    hmmwv.GetSystem().Add(box)

    # Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVerticalDir(ch.CameraVerticalDir_Z)  # Fixed method name
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain with Sensors')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    # Updated camera position
    vis.AddCamera(ch.ChVectorD(-5, 2.5, 1.5), ch.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVectorD(1.5, -2.5, 5.5), ch.ChVectorD(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Driver initialization with corrected parameters
    driver = veh.ChDriver(hmmwv)  # Removed unnecessary GetVehicle()
    driver.Initialize(hmmwv.GetSystem())  # Added system parameter

    # ROS setup
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    # Sensor setup
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())  # Sensor manager initialization

    # Lidar sensor configuration
    lidar = sens.ChLidarSensor()
    lidar.SetName("ch_lidar")
    lidar.SetBody(hmmwv.GetChassisBody())  # Attach to chassis
    lidar.SetPosition(ch.ChVectorD(0, 0, 1.5))  # Position relative to chassis
    lidar.SetRange(50)
    lidar.SetHorizontalResolution(0.5)
    lidar.SetVerticalResolution(0.5)
    lidar.SetFOVHorizontal(180)
    lidar.SetFOVVertical(30)
    lidar.SetUpdateRate(10)  # 10Hz
    # Add filters
    voxel_filter = sens.ChVoxelGridFilter()
    voxel_filter.SetVoxelSize(0.1)
    lidar.AddFilter(voxel_filter)
    sens_manager.AddSensor(lidar)

    # ROS Lidar handler registration
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(10, lidar, "~/output/lidar"))  # Added ROS Lidar handler

    ros_manager.Initialize()

    # Simulation parameters
    time_step = 1e-3
    time_end = 30
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    step_number = 0
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
        sens_manager.Update(time_step)  # Added sensor update
        
        if not ros_manager.Update(time, time_step):
            break
        step_number += 1

if __name__ == "__main__":
    main()