import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truck_initLoc = chrono.ChVectorD(0, 0, 1)  
truck_initRot = chrono.ChQuaternionD(chrono.Q_from_AngAxis(chrono.CH_C_PI / 6, chrono.ChVectorD(0,1,0)))  


sedan_initLoc = chrono.ChVectorD(5, 0, 0.5)  
sedan_initRot = chrono.ChQuaternionD(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


truck_chassis_collision_type = veh.CollisionType_NONE
sedan_chassis_collision_type = veh.CollisionType_PRIMITIVES


truck_tire_model = veh.TireModelType_RIGID  
sedan_tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


contact_method = chrono.ChContactMethod_NSC
step_size = 1e-3
render_step_size = 1.0 / 50


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(truck_chassis_collision_type)
truck.SetInitPosition(chrono.ChCoordsysD(truck_initLoc, truck_initRot))


for axle in truck.GetAxles():
    axle.GetTire().SetType(truck_tire_model)

truck.Initialize()
truck.SetChassisVisualizationType(vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type)
truck.SetWheelVisualizationType(vis_type)
truck.SetTireVisualizationType(vis_type)


sedan = veh.Jeep()  
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(sedan_chassis_collision_type)
sedan.SetInitPosition(chrono.ChCoordsysD(sedan_initLoc, sedan_initRot))
sedan.SetTireType(sedan_tire_model)  
sedan.Initialize()
sedan.SetChassisVisualizationType(vis_type)
sedan.SetSteeringVisualizationType(vis_type)
sedan.SetSuspensionVisualizationType(vis_type)
sedan.SetWheelVisualizationType(vis_type)
sedan.SetTireVisualizationType(vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())  
patch = terrain.AddPatch(patch_mat,
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/highway.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Truck and Sedan Simulation')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(truck.GetTractor().COM express(), 25.0, 1.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan)  


truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()


sedan_driver = veh.ChDriver()
sedan_driver.SetThrottle(0.5)  
sedan_driver.SetSteering(0)    


truck_states = []
sedan_states = []


step_number = 0
render_frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = truck.GetSystem().GetChTime()
    
    
    if step_number % (render_step_size / step_size) == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    
    
    truck_inputs = truck_driver.GetInputs()
    
    
    sedan_inputs = sedan_driver.GetInputs()
    
    
    truck_driver.Synchronize(time)
    terrain.Synchronize(time)
    truck.Synchronize(time, truck_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, truck_inputs)
    
    
    truck_driver.Advance(step_size)
    terrain.Advance(step_size)
    truck.Advance(step_size)
    sedan.Advance(step_size)
    vis.Advance(step_size)
    
    
    truck_states.append(truck.GetTractor().GetPos())
    if hasattr(truck, 'GetTrailer'):
        trailer_pos = truck.GetTrailer().GetPos()
        truck_states[-1] = (truck_states[-1], trailer_pos)
    sedan_states.append(sedan.GetChassis().GetPos())
    
    step_number += 1
    realtime_timer.Spin(step_size)


print("Truck tractor positions:", truck_states)
print("Sedan positions:", sedan_states)