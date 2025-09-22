import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import random
import time


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
vehicle.SetMass(1000)  

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


terrain.AddMovingPatch(vehicle.GetChassisBody(), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(5, 3, 1))


terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)


terrain.Initialize(20, 20, 0.02)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo with Objects and Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


light_positions = [
    chrono.ChVector3d(0, 0, 5),
    chrono.ChVector3d(10, 10, 5),
    chrono.ChVector3d(-5, -5, 5),
    chrono.ChVector3d(5, -5, 5)
]
for pos in light_positions:
    vis.AddLightDirectional(pos, 0, 1, 1, 1)


sensor_manager = veh.ChSensorManager()
camera_sensor = veh.ChCameraSensor()
camera_sensor.SetResolution(1280, 720)
camera_sensor.SetFieldOfView(45.0, 45.0)
camera_sensor.SetPosition(trackPoint)
camera_sensor.AttachToSensorManager(sensor_manager)
sensor_manager.AddSensor(camera_sensor)


vis.AddFilter(chrono.ChFilter_CameraSensor, camera_sensor)


driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)  
driver.SetThrottleDelta(render_step_size / 1.0)  
driver.SetBrakingDelta(render_step_size / 0.3)  



def AddRandomBox():
    box_size = chrono.ChVector3d(random.uniform(0.5, 1.5), random.uniform(0.5, 1.5), 0.5)
    box_pos = chrono.ChVector3d(
        random.uniform(-10, 10),
        random.uniform(-10, 10),
        random.uniform(0, 2)
    )
    box_mass = random.uniform(1, 10)
    
    
    box = veh.ChBodyEasyBox(
        box_size.X,
        box_size.Y,
        box_size.Z,
        box_mass,
        visualize=True,
        collision=True,
        material=chrono.ChContactMaterialSMC()
    )
    
    
    box.SetPos(box_pos)
    vehicle.GetSystem().Add(box)
    
    
    box_shape = chrono.ChVisualShapeBox(box_size)
    box_shape.SetColor(chrono.ChColor(0, 1, 0))  
    box.AddVisualShape(box_shape)


for _ in range(20):
    AddRandomBox()


driver.Initialize()


render_steps = math.ceil(render_step_size / step_size)


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

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

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)