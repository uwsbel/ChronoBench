import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(-8, 0, 0.6)
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


render_step_size = 1.0 / 50  


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
terrain.Initialize(20, 20, 0.02)


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


def add_random_box(system, vehicle_pos, min_distance=5):
    while True:
        x = random.uniform(-50, 50)
        y = random.uniform(-50, 50)
        z = terrainHeight + 2
        pos = chrono.ChVector3d(x, y, z)
        
        
        if (pos - vehicle_pos).Length() > min_distance:
            break
    
    box = chrono.ChBodyEasyBox(1, 1, 1, 1000)
    box.SetPos(pos)
    box.SetMaterialSurface(chrono.ChMaterialSurface())
    box.GetMaterialSurface().SetFriction(0.5)
    box.GetMaterialSurface().SetDampingF(0.2)
    box.SetBodyFixed(True)
    
    
    color = chrono.ChColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
    box.SetVisualization(chrono.ChVisualizationMaterialColor(color))
    
    system.Add(box)


for _ in range(20):
    add_random_box(vehicle.GetSystem(), vehicle.GetChassisBody().GetPos())


sensor_manager = chrono.ChSensorManager()
sensor_manager.AttachTo(vehicle.GetChassisBody())


light1 = chrono.ChPointLight()
light1.SetPos(chrono.ChVector3d(10, 10, 10))
light1.SetIntensity(100)
sensor_manager.AddLight(light1)

light2 = chrono.ChPointLight()
light2.SetPos(chrono.ChVector3d(-10, -10, 10))
light2.SetIntensity(100)
sensor_manager.AddLight(light2)


camera = chrono.ChCameraSensor()
camera.SetResolution(640, 480)
camera.SetFieldOfView(60)
camera.SetPosition(chrono.ChVector3d(0, 0, 1.71))  
camera.SetRotation(chrono.ChQuaterniond(1, 0, 0, 0))
sensor_manager.AddSensor(camera)


filter = chrono.ChFilterLaneDetection()
filter.SetInputSensor(camera)
sensor_manager.AddFilter(filter)


vehicle.GetSystem().Add(sensor_manager)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.RenderSensorManager(sensor_manager)  
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    sensor_manager.Synchronize(time)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)