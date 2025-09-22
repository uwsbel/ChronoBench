import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath('/path/to/chrono/data/')
veh.SetDataPath('/path/to/chrono/data/vehicle/')


initLoc_truck = chrono.ChVector3d(0, 0, 0.5)
initRot_truck = chrono.ChQuaterniond(1, 0, 0, 0)
initLoc_sedan = chrono.ChVector3d(10, 0, 0.5)
initRot_sedan = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_RIGID  


terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


contact_method = chrono.ChContactMethod_NSC
contact_vis = False


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(initLoc_truck, initRot_truck))
truck.Initialize()


truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)
truck.GetTractor().SetTireType(tire_model)  


sedan = veh.CityBus()  
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(initLoc_sedan, initRot_sedan))
sedan.Initialize()

sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


terrain = veh.RigidTerrain(truck.GetSystem())
mesh_file = veh.GetDataFile("terrain/mesh.obj")  
patch_mat = chrono.ChContactMaterialNSC()
patch = terrain.AddPatch(patch_mat, mesh_file, 1.0, 1.0, chrono.ChVector3d(0, 0, 0), chrono.QUNIT)
terrain.Initialize()


vis_truck = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_truck.SetWindowTitle('Kraz Demo')
vis_truck.SetWindowSize(1280, 1024)
vis_truck.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)
vis_truck.Initialize()
vis_truck.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_truck.AddLightDirectional()
vis_truck.AddSkyBox()
vis_truck.AttachVehicle(truck.GetTractor())

vis_sedan = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis_sedan.SetWindowTitle('Sedan Demo')
vis_sedan.SetWindowSize(1280, 1024)
vis_sedan.SetChaseCamera(chrono.ChVector3d(0, 0, 2.1), 25.0, 1.5)
vis_sedan.Initialize()
vis_sedan.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis_sedan.AddLightDirectional()
vis_sedan.AddSkyBox()
vis_sedan.AttachVehicle(sedan.GetVehicle())


driver_truck = veh.ChInteractiveDriverIRR(vis_truck)
driver_sedan = veh.ChDriver(sedan.GetVehicle())  


driver_truck.Initialize()
driver_sedan.Initialize()


print("TRUCK MASS: ", truck.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0


tractor_states = []
trailer_states = []

while vis_truck.Run() and vis_sedan.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis_truck.BeginScene()
        vis_truck.Render()
        vis_truck.EndScene()
        
        vis_sedan.BeginScene()
        vis_sedan.Render()
        vis_sedan.EndScene()
        
        render_frame += 1

    
    driver_inputs_truck = driver_truck.GetInputs()
    driver_inputs_sedan = veh.Inputs()  
    driver_inputs_sedan.m_throttle = 0.5  
    driver_inputs_sedan.m_steering = 0  

    
    driver_truck.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, driver_inputs_truck, terrain)
    vis_truck.Synchronize(time, driver_inputs_truck)
    
    driver_sedan.Synchronize(time)
    sedan.Synchronize(time, driver_inputs_sedan, terrain)
    vis_sedan.Synchronize(time, driver_inputs_sedan)

    
    driver_truck.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    vis_truck.Advance(step_size)
    
    driver_sedan.Advance(step_size)
    sedan.Advance(step_size)
    vis_sedan.Advance(step_size)

    
    tractor_states.append(truck.GetTractor().GetPos())
    trailer_states.append(truck.GetTrailer().GetPos())

    
    step_number += 1

    
    realtime_timer.Spin(step_size)