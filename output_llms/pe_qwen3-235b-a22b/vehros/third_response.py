import pychrono as ch
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.sensor as sens  # Added for sensor functionalities
from pychrono import irrlicht as chronoirr
import math

def main():
    # Set Chrono data path
    ch.SetChronoDataPath(ch.GetChronoDataPath())
    veh.SetDataPath(ch.GetChronoDataPath() + 'vehicle/')
    
    # Create the HMMWV vehicle
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
    
    # Set visualization types
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
    patch = terrain.AddPatch(patch_mat, ch.CSYSNORM, 100.0, 100.0)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
    terrain.Initialize()

    # Add visualization box
    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, patch_mat)
    box.SetPos(ch.ChVector3d(0, 0, 0.5))
    hmmwv.GetSystem().Add(box)

    # Create visualization system
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(hmmwv.GetSystem())
    vis.SetCameraVertical(chronoirr.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Viper rover - Rigid terrain with Sensor')
    vis.Initialize()
    vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 0.5))  # Updated camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)

    # Create driver system
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()

    # Create sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetChassisBody())
    
    # Create and configure lidar sensor
    lidar = sens.ChLidarSensor(
        hmmwv.GetChassisBody(),
        100,  # Update rate
        ch.ChFrameD(ch.ChVector3d(0, 0, 2), ch.ChQuaterniond(1, 0, 0, 0)),
        360,  # Horizontal samples
        1,    # Vertical samples
        math.radians(360),  # Horizontal FOV
        0.01, # Minimum distance
        100   # Maximum distance
    )
    lidar.AddFilter(sens.ChFilterD_noise(sens.ChFilterNoiseNone()))
    lidar.AddFilter(sens.ChFilterD_return(sens.ChFilterReturnValid()))
    lidar.AddFilter(sens.ChFilterD_visualize(100, "Lidar Data"))
    sens_manager.AddSensor(lidar)

    # Create ROS manager
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyStateHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))  # Register lidar handler
    ros_manager.Initialize()

    # Simulation loop
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        
        # Render scene
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
        
        # Synchronize systems
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        
        # Advance systems
        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        
        # Update sensor manager
        sens_manager.Update()
        
        # Update ROS
        if not ros_manager.Update(time, time_step):
            break
            
        step_number += 1

if __name__ == "__main__":
    main()

import pychrono.sensor as sens

vis.SetCameraVertical(chronoirr.CameraVerticalDir_Z)

ros_manager.RegisterHandler(chros.ChROSBodyStateHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))

box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, patch_mat)
   box.SetPos(ch.ChVector3d(0, 0, 0.5))
   hmmwv.GetSystem().Add(box)

sens_manager = sens.ChSensorManager(hmmwv.GetChassisBody())
   lidar = sens.ChLidarSensor(...)
   lidar.AddFilter(...)
   sens_manager.AddSensor(lidar)

ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/lidar"))

vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 0.5))

sens_manager.Update()