import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-15, 0, 1.2)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 20


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


heightmap_file = veh.GetDataFile("terrain/height_maps/bump64.bmp")
terrain.Initialize(heightmap_file, 40, 40, 40/(64-1), 40/(64-1), 0.02)
terrain.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)


np.random.seed(42)
for _ in range(5):
    pos = chrono.ChVector3d(np.random.uniform(-20, 20), np.random.uniform(-20, 20), 0.5)
    box = chrono.ChBody()
    box.SetPos(pos)
    box.SetMass(1000)
    box.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    box.SetBodyFixed(True)
    
    box_shape = chrono.ChBoxShape(chrono.ChVector3d(1, 1, 1))
    box_shape.SetColor(chrono.ChColor(0.8, 0.5, 0.0))
    box.AddVisualShape(box_shape)
    
    material = chrono.ChContactMaterialSMC()
    collision_shape = chrono.ChCollisionShapeBox(material, 1, 1, 1)
    box.AddCollisionShape(collision_shape)
    box.SetCollide(True)
    
    vehicle.GetSystem().Add(box)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())


lidar_offset = chrono.ChVector3d(0, 0, 2)
lidar_pose = chrono.ChFrameD(lidar_offset, chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0)))

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),
    10,
    lidar_pose,
    900,
    30,
    chrono.CH_PI * 2,
    chrono.CH_PI / 6,
    100.0,
    sens.LidarReturnMode_FIRST_RETURN,
    sens.LidarModelType_RAYCAST
)
lidar.SetName("Vehicle Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(0.0)


lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVoxelFilter(0.1, 0.1, 0.1))
lidar.PushFilter(sens.ChFilterVisualize(900, 30, "Lidar Output"))

sensor_manager.AddSensor(lidar)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    sensor_manager.Update()
    
    step_number += 1
    realtime_timer.Spin(step_size)