import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')




















initLoc_truck = chrono.ChVector3d(0, 0, 0.5)  
initLoc_truck = chrono.ChVector3d(0, 0, 0.7)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)  


initLoc_sedan = chrono.ChVector3d(5, 2, 0.5)  
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID


tire_model_sedan = veh.TireModelType_RIGID


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


mesh_file = veh.GetDataFile("terrain/mesh/broken_highway.obj")
height_map_file = veh.GetDataFile("terrain/mesh/broken_highway_height.png")

texture_file = veh.GetDataFile("terrain/textures/asphalt.jpg")

mesh_terrain = veh.TerrainMesh(vehicle=None)

mesh_terrain.SetMeshFilename(mesh_file)
mesh_terrain.SetTextureFilename(texture_file)
mesh_terrain.Initialize()




vehicle_truck = veh.Kraz()
vehicle_truck.SetContactMethod(contact_method)
vehicle_truck.SetChassisCollisionType(chassis_collision_type)
vehicle_truck.SetChassisFixed(False)
vehicle_truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
vehicle_truck.Initialize()

vehicle_truck.SetChassisVisualizationType(vis_type, vis_type)
vehicle_truck.SetSteeringVisualizationType(vis_type)
vehicle_truck.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle_truck.SetWheelVisualizationType(vis_type, vis_type)
vehicle_truck.SetTireVisualizationType(vis_type, vis_type)

vehicle_truck.SetTireType(tire_model_truck)  

vehicle_truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)

sedan.SetTireType(tire_model_sedan)  

sedan.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Kraz and Sedan Demo')
vis.SetWindowSize(1280, 1024)


trackPoint = chrono.ChVector3d(0, 0, 2.1)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()

vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()


vis.AttachVehicle(vehicle_truck.GetVehicle())  
vis.AttachVehicle(sedan.GetVehicle())



driver_truck = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver_truck.SetSteeringDelta(render_step_size / steering_time)
driver_truck.SetThrottleDelta(render_step_size / throttle_time)
driver_truck.SetBrakingDelta(render_step_size / braking_time)
driver_truck.Initialize()


class SimpleSedanDriver:
    def __init__(self):
        
        self.throttle = 0.6  
        self.steering = 0.0  
        self.braking = 0.0
    
    def GetInputs(self):
        return veh.DriverInputs(self.steering, self.throttle, self.braking)
    
    def Synchronize(self, time):
        pass
    
    def Advance(self, step):
        pass

driver_sedan = SimpleSedanDriver()


print("TRUCK MASS: ", vehicle_truck.GetVehicle().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = int(math.ceil(render_step_size / step_size))


realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
render_frame = 0




truck_chassis = vehicle_truck.GetVehicle()

try:
    truck_trailer = vehicle_truck.GetTrailer()
except AttributeError:
    truck_trailer = None


truck_state_storage = []


while vis.Run():
    time = vehicle_truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    driver_sedan.Synchronize(time)
    
    
    mesh_terrain.Advance(step_size)

    vehicle_truck.Synchronize(time, driver_inputs_truck, mesh_terrain)
    sedan.Synchronize(time, driver_inputs_sedan, mesh_terrain)
    vis.Synchronize(time, driver_inputs_truck)  

    
    driver_truck.Advance(step_size)
    driver_sedan.Advance(step_size)
    mesh_terrain.Advance(step_size)
    vehicle_truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    
    chassis_pos = truck_chassis.GetPos()
    chassis_rot = truck_chassis.GetRot()
    state = {
        'time': time,
        'chassis_pos': chrono.ChVector3d(chassis_pos),
        'chassis_rot': chrono.ChQuaterniond(chassis_rot)
    }

    if truck_trailer is not None:
        trailer_pos = truck_trailer.GetPos()
        trailer_rot = truck_trailer.GetRot()
        state['trailer_pos'] = chrono.ChVector3d(trailer_pos)
        state['trailer_rot'] = chrono.ChQuaterniond(trailer_rot)
    else:
        state['trailer_pos'] = None
        state['trailer_rot'] = None

    truck_state_storage.append(state)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)