import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVectorD(-8, 0, 0.6)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID


terrainHeight = 0  
terrainLength = 100.0  
terrainWidth = 100.0  


trackPoint = chrono.ChVectorD(0.0, 0.0, 1.71)


contact_method = chrono.ChContactMethod_SMC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
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
terrain.SetSoilParameters(2e6,  
                          0,  
                          1.1,  
                          0,  
                          30,  
                          0.01,  
                          2e8,  
                          3e4  
)


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))


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


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3  
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


sensor_manager = chrono.ChSensorManager()


light1 = chrono.ChLightPoint()
light1.SetPosition(chrono.ChVectorD(10, 0, 5))
light1.SetColor(chrono.ChColor(1, 1, 1))
light1.SetIntensity(10)
sensor_manager.AddSensor(light1)

light2 = chrono.ChLightPoint()
light2.SetPosition(chrono.ChVectorD(-10, 0, 5))
light2.SetColor(chrono.ChColor(1, 1, 1))
light2.SetIntensity(10)
sensor_manager.AddSensor(light2)


camera = chrono.ChCameraSensor()
camera.SetPosition(chrono.ChVectorD(0, 0, 2))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))
camera.SetResolution(640, 480)
camera.SetFieldOfView(45)
camera.SetNearClipDistance(0.1)
camera.SetFarClipDistance(100)
camera.SetAspectRatio(1.333)
vehicle.GetChassisBody().AddSensor(camera)


filter = chrono.ChFilterCamera()
filter.SetCamera(camera)
filter.SetWindowName('Camera Feed')
filter.SetWindowSize(640, 480)
sensor_manager.AddFilter(filter)


for _ in range(10):
    box = chrono.ChBodyEasyBox(vehicle.GetSystem(), 1, 1, 1, 1000, True, True)
    box.SetPos(chrono.ChVectorD(random.uniform(-10, 10), random.uniform(-10, 10), 1))
    box.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    box.SetMaterial(chrono.ChMaterialSurfaceNSC())
    vehicle.GetSystem().Add(box)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
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