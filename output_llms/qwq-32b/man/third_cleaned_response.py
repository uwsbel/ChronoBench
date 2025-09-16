import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensors  
import numpy as np  
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVectorD(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordSysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystemType.BULLET)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat,
                        chrono.ChCoordSysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                        terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


def add_random_box(system, terrain_height):
    x = np.random.uniform(-terrainLength/2 + 5, terrainLength/2 -5)
    y = np.random.uniform(-terrainWidth/2 +5, terrainWidth/2 -5)
    z = terrain_height + 1.0  
    pos = chrono.ChVectorD(x, y, z)
    
    box = chrono.ChBodyEasyBox(1, 1, 1, 2000, True, True)
    box.SetPos(pos)
    box.SetBodyFixed(False)
    system.Add(box)
    
    box.GetCollisionModel().ClearModel()
    box.GetCollisionModel().AddBox(0.5, 0.5, 0.5)
    box.GetCollisionModel().BuildModel()
    box.SetCollide(True)
    
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(1,1,1))
    box.AddAsset(box_shape)
    box.AddAsset(chrono.ChColorAsset(0.5, 0, 0))

for _ in range(10):
    add_random_box(vehicle.GetSystem(), terrainHeight)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize(vehicle.GetVehicle())  


sensor_manager = vehicle.GetSystem().GetSensorManager()
lidar = sensors.ChLidar()
lidar.SetName("lidar")
lidar.SetPosition(chrono.ChVectorD(0, 0, 1.5))  
lidar.SetRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0)))  
lidar.SetBeamAperture(chrono.ChVectorD(chrono.CH_C_PI/6, chrono.CH_C_PI/6))
lidar.SetBeamSpacing(chrono.ChVectorD(chrono.CH_C_PI/180, chrono.CH_C_PI/180))
lidar.SetRange(100)
lidar.SetSamplingRate(0.1)
lidar.SetNoise(0.05)
sensor_manager.AddSensor(lidar)


print("VEHICLE MASS: ", vehicle.GetMass())


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()
    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    driver_inputs = driver.GetInputs()
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    
    
    sensor_manager.Advance(step_size)
    
    step_number += 1
    realtime_timer.Spin(step_size)