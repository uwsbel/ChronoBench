import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


initLocTruck = chrono.ChVector3d(-5, 0, 0.5)
initRotTruck = chrono.ChQuaterniond(1, 0, 0, 0)
initLocSedan = chrono.ChVector3d(5, 0, 0.5)
initRotSedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model_truck = veh.TireModelType_RIGID  
tire_model_sedan = veh.TireModelType_TMEASY


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLocTruck, initRotTruck))
truck.Initialize()
truck.SetTireType(tire_model_truck)


truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)


sedan = veh.CityBus()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLocSedan, initRotSedan))
sedan.Initialize()
sedan.SetTireType(tire_model_sedan)


sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


terrain = veh.RigidTerrain(truck.GetSystem())
mesh_file = veh.GetDataFile("terrain/meshes/highway.obj")
mesh_mat = chrono.ChContactMaterialNSC()
mesh_mat.SetFriction(0.9)
mesh_mat.SetRestitution(0.01)
patch = terrain.AddMesh(mesh_mat, mesh_file, 1.0, chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Multi-Vehicle Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetVehicle())


driver_truck = veh.ChInteractiveDriverIRR(vis)
driver_truck.SetSteeringDelta(render_step_size / 1.0)
driver_truck.SetThrottleDelta(render_step_size / 1.0)
driver_truck.SetBrakingDelta(render_step_size / 0.3)
driver_truck.Initialize()


class SimpleDriver:
    def __init__(self, throttle, steering):
        self.throttle = throttle
        self.steering = steering

    def GetInputs(self):
        inputs = veh.DriverInputs()
        inputs.m_throttle = self.throttle
        inputs.m_steering = self.steering
        inputs.m_braking = 0
        return inputs

driver_sedan = SimpleDriver(0.3, 0.0)  


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

truck_states = []

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = driver_sedan.GetInputs()

    
    driver_truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis.Synchronize(time, driver_inputs_truck)

    
    truck_state = {
        'time': time,
        'tractor_pos': truck.GetTractor().GetPos(),
        'trailer_pos': truck.GetTrailer().GetPos() if hasattr(truck, 'GetTrailer') else None
    }
    truck_states.append(truck_state)

    
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)


for state in truck_states:
    print(state)