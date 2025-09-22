import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

# Initialize system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create terrain
terrain = veh.RigidTerrain(system,
                          chrono.ChVectorD(0, 1, 0),  # normal vector (upwards)
                          chrono.ChVectorD(100, 0.1, 100),  # terrain dimensions (x, y, z)
                          False,  # no tire-track interaction
                          veh.material_tire_terrain)

# Create Gator vehicle
vehicle = veh.Gator(terrain, system)
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
vehicle.SetTireVisualizationType(veh.VisualizationType.NONE)

# Initialize driver
driver = veh.ChInteractiveDriver()
vehicle.SetDriver(driver)

# Initialize sensor manager
sensor_mgr = veh.ChSensorManager(system)

# Add camera sensor
cam_pos = chrono.ChVectorD(0, 0.5, 0)  # Relative to chassis
cam_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # Facing forward
camera = sens.ChCameraSensor(vehicle.GetChassis(), cam_pos, cam_rot)
camera.SetFov(60)
camera.SetImageWidth(800)
camera.SetImageHeight(600)
sensor_mgr.AddSensor(camera)

# Add point lights
light1 = sens.ChLight()
light1.SetPosition(chrono.ChVectorD(5, 5, 5))
light1.SetIntensity(200)
sensor_mgr.AddLight(light1)

light2 = sens.ChLight()
light2.SetPosition(chrono.ChVectorD(-5, 5, 5))
light2.SetIntensity(200)
sensor_mgr.AddLight(light2)

# Setup Irrlicht visualization
my_vis = irr.ChIrrApp(system, 'Gator Simulation', irr.dimension2du(1024, 768))
my_vis.AddTypicalLights()
my_vis.AddTypicalLogo()
my_vis.AddTypicalSky()
my_vis.SetCamera(chronoirr.vector3df(0, 0, 5))  # Initial camera position

# Add assets to Irrlicht scene
my_vis.AddAsset(terrain.GetGround().GetVisualizationAsset())
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.PRIMITIVES)
my_vis.AssetCollector.Add(vehicle.GetVisualizationAssets())

# Simulation loop
my_vis.SetTimestep(0.01)

while my_vis.Run():
    my_vis.BeginScene()
    my_vis.Render()
    my_vis.EndScene()

    # Update driver inputs
    driver.Synchronize(my_vis.GetSteering(), my_vis.GetThrottle(), my_vis.GetBraking())

    # Update vehicle
    vehicle.Update()

    # Update sensor manager
    current_time = system.GetChTime()
    time_step = my_vis.GetTimestep()
    sensor_mgr.Update(current_time, time_step)

    # Advance simulation
    system.DoStepDynamics(time_step)