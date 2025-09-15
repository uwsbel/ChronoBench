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
    
    # Create the terrain for the vehicle to interact with.
    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch_mat = ch.ChContactMaterialNSC()
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
    vis.AddCamera(ch.ChVector3d(-5, 2.5, 1.5), ch.ChVector3d(0, 0, 1))  # Updated camera position
    vis.AddTypicalLights()
    vis.AddLightWithShadow(ch.ChVector3d(1.5, -2.5, 5.5), ch.ChVector3d(0, 0, 0.5), 3, 4, 10, 40, 512)
    
    # Add visualization box
    box = sens.ChBodyEasyBox(1, 1, 1)
    box.SetPos(ch.ChVector3d(5, 0, 1.6))
    box.GetVisualShape(0).SetMaterialColor(ch.ChColor(0.5, 0.5, 0.5))
    vis.GetSystem().AddBody(box)
    
    # Create and initialize the driver system.
    driver = veh.ChDriver(hmmwv.GetVehicle())
    driver.Initialize()
    
    # Create the ROS manager and register handlers for communication.
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSDriverInputsHandler(25, driver, "~/input/driver_inputs"))
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, hmmwv.GetChassisBody(), "~/output/hmmwv/state"))
    
    # Set up sensor manager
    sens_manager = sens.ChSensorManager(hmmwv.GetSystem())
    
    # Add LiDAR sensor
    lidar = sens.ChLidarSensor()
    lidar.SetName("lidar")
    lidar.SetPosition(ch.ChVector3d(0, 0, 1.6))
    lidar.SetRotation(ch.ChQuaterniond(1, 0, 0, 0))
    lidar.SetRange(50.0)
    lidar.SetHorizontalAngleRange(-1.5708, 1.5708)
    lidar.SetVerticalAngleRange(-0.7854, 0.7854)
    lidar.SetNumberOfBeams(360)
    lidar.SetBeamSpacing(0.01)
    
    # Add filters to LiDAR
    distance_filter = sens.ChLidarDistanceFilter()
    distance_filter.SetMinDistance(0.1)
    distance_filter.SetMaxDistance(50.0)
    lidar.AddFilter(distance_filter)
    
    intensity_filter = sens.ChLidarIntensityFilter()
    intensity_filter.SetMinIntensity(0.1)
    intensity_filter.SetMaxIntensity(1.0)
    lidar.AddFilter(intensity_filter)
    
    coordinate_filter = sens.ChLidarCoordinateFilter()
    coordinate_filter.SetMinX(-10)
    coordinate_filter.SetMaxX(10)
    coordinate_filter.SetMinY(-10)
    coordinate_filter.SetMaxY(10)
    coordinate_filter.SetMinZ(0)
    coordinate_filter.SetMaxZ(20)
    lidar.AddFilter(coordinate_filter)
    
    sens_manager.AddSensor(lidar)
    
    # Register ROS lidar handler
    ros_manager.RegisterHandler(chros.ChROSLidarHandler(25, lidar, "~/output/hmmwv/lidar"))
    
    ros_manager.Initialize()
    
    # Start the simulation loop.
    time = 0
    time_step = 1e-3
    time_end = 30
    step_number = 0
    render_step_size = 1.0 / 25
    render_steps = math.ceil(render_step_size / time_step)
    hmmwv.GetVehicle().EnableRealtime(True)
    
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()
        
        if (step_number % render_steps == 0):
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
        
        sens_manager.Update()  # Update sensor manager
        
        if not ros_manager.Update(time, time_step):
            break
            
        step_number += 1

if __name__ == "__main__":
    main()