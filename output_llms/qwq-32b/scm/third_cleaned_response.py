import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensors as sens
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
step_size = 1e-3
render_step_size = 1.0 / 50


vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()
vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)


terrain = veh.SCMTerrain(vehicle.GetSystem())
terrain.SetSoilParameters(2e6, 0, 1.1, 0, 30, 0.01, 2e8, 3e4)
terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)

terrain.Initialize(20, 20, 1000, 1000)  


num_boxes = 10
for _ in range(num_boxes):
    x = random.uniform(-40, 40)
    y = random.uniform(-20, 20)
    z = terrainHeight + 0.1  
    box = chrono.ChBodyEasyBox(1, 1, 0.5, 1000)  
    box.SetPos(chrono.ChVectorD(x, y, z))
    vehicle.GetSystem().Add(box)


sensor_manager = sens.ChSensorManager(vehicle.GetSystem())
camera = sens.ChCameraSensor()
camera.SetName("MainCamera")
camera.SetPosition(chrono.ChVectorD(0, 0, 1.5))  
camera.SetDirection(chrono.ChVectorD(0, -1, 0))
camera.SetFOV(60)
camera.SetImageResolution(800, 600)
camera.SetAttachmentFrame(vehicle.GetChassisBody())
sensor_manager.AddSensor(camera)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLightPoint(chrono.ChVectorD(20, 0, 10), chrono.ChColor(1, 1, 1), 50)
vis.AddLightPoint(chrono.ChVectorD(-20, 0, 10), chrono.ChColor(1, 1, 1), 50)
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()


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
        
        camera.ExportImage(f'output/camera_{render_frame:04d}.png')
    
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