import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sensor
import numpy as np


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


vehicle = veh.MAN_10t() 
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


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)


patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


def AddRandomBoxes(terrain, num_boxes=10, min_size=0.5, max_size=2.0):
    for _ in range(num_boxes):
        
        size = np.random.uniform(min_size, max_size)
        pos = chrono.ChVector3d(
            np.random.uniform(-50, 50),
            np.random.uniform(-50, 50),
            np.random.uniform(0, 5)
        )
        mat = chrono.ChContactMaterialNSC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.01)
        box = terrain.AddBox(
            mat,
            pos,
            size,
            size,
            size
        )
        box.SetColor(chrono.ChColor(np.random.uniform(0.5, 1.0), np.random.uniform(0.5, 1.0), np.random.uniform(0.5, 1.0)))
        box.Initialize()


class SensorManager:
    def __init__(self):
        self.sensors = []
        
    def AddSensor(self, sensor):
        self.sensors.append(sensor)
        
    def Synchronize(self, time):
        for sensor in self.sensors:
            sensor.Synchronize(time)

class LidarSensor:
    def __init__(self, vehicle_system):
        self.vehicle = vehicle
        self.system = vehicle_system
        self.sensor = None
        
    def Initialize(self):
        self.sensor = self.system.CreateLidarSensor(
            chrono.ChVector3d(0, 0, 0),
            100.0,
            0.5,
            0.1
        )
        self.sensor.Initialize()
        
    def GetData(self):
        return self.sensor.GetData()
        
    def Update(self, time):
        self.sensor.Update(time)


sensor_manager = SensorManager()
lidar_sensor = LidarSensor(vehicle.GetSystem())
lidar_sensor.Initialize()
sensor_manager.AddSensor(lidar_sensor)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo with Sensors')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
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


print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

def AddRandomBoxesToTerrain(terrain):
    AddRandomBoxes(terrain, 10, 0.5, 2.0)


AddRandomBoxesToTerrain(terrain)

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
    
    
    step_number += 1
    
    
    realtime_timer.Spin(step_size)
    
    
    lidar_sensor.Update(time)
    
    
    
    
    
    print(f"Frame: {render_frame} | Time: {time}")