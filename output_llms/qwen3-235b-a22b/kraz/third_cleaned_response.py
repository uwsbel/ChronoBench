import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


truckInitLoc = chrono.ChVector3d(-20, 0, 0.5)
truckInitRot = chrono.ChQuaterniond(1, 0, 0, 0)
sedanInitLoc = chrono.ChVector3d(0, 5, 0.5)
sedanInitRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


truck_tire_model = veh.TireModelType_RIGID
sedan_tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(0, 0, 2.1)


contact_method = chrono.ChContactMethod_NSC


step_size = 1e-3
render_step_size = 1.0 / 50


truck = veh.Kraz()
truck.SetContactMethod(contact_method)
truck.SetChassisCollisionType(chassis_collision_type)
truck.SetChassisFixed(False)
truck.SetInitPosition(chrono.ChCoordsysd(truckInitLoc, truckInitRot))
truck.Initialize()
truck.SetTireType(truck_tire_model)  
truck.SetChassisVisualizationType(vis_type, vis_type)
truck.SetSteeringVisualizationType(vis_type)
truck.SetSuspensionVisualizationType(vis_type, vis_type)
truck.SetWheelVisualizationType(vis_type, vis_type)
truck.SetTireVisualizationType(vis_type, vis_type)
truck.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


sedan = veh.Sedan()
sedan.SetContactMethod(contact_method)
sedan.SetChassisCollisionType(chassis_collision_type)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(chrono.ChCoordsysd(sedanInitLoc, sedanInitRot))
sedan.Initialize()
sedan.SetTireType(sedan_tire_model)
sedan.SetChassisVisualizationType(vis_type, vis_type)
sedan.SetSuspensionVisualizationType(vis_type, vis_type)
sedan.SetWheelVisualizationType(vis_type, vis_type)
sedan.SetTireVisualizationType(vis_type, vis_type)


patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(truck.GetSystem())
patch = terrain.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    veh.GetDataFile("terrain/meshes/highway.obj"), True, 0.01)
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
vis.AttachVehicle(truck.GetTractor())
vis.AttachVehicle(sedan.GetVehicle())


truck_driver = veh.ChInteractiveDriverIRR(vis)
truck_driver.SetSteeringDelta(render_step_size / 1.0)
truck_driver.SetThrottleDelta(render_step_size / 1.0)
truck_driver.SetBrakingDelta(render_step_size / 0.3)
truck_driver.Initialize()


print("TRUCK MASS: ", truck.GetTractor().GetMass())
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())


render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


with open('truck_states.csv', 'w') as f:
    f.write("Time,TractorPosX,TractorPosY,TractorPosZ,TractorRotW,TractorRotX,TractorRotY,TractorRotZ,")
    f.write("TrailerPosX,TrailerPosY,TrailerPosZ,TrailerRotW,TrailerRotX,TrailerRotY,TrailerRotZ\n")

while vis.Run():
    time = truck.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        
        
        tractor = truck.GetTractor()
        trailer = truck.GetTrailer()
        with open('truck_states.csv', 'a') as f:
            f.write(f"{time},{tractor.GetPos().x},{tractor.GetPos().y},{tractor.GetPos().z},")
            f.write(f"{tractor.GetRot().e0},{tractor.GetRot().e1},{tractor.GetRot().e2},{tractor.GetRot().e3},")
            f.write(f"{trailer.GetPos().x},{trailer.GetPos().y},{trailer.GetPos().z},")
            f.write(f"{trailer.GetRot().e0},{trailer.GetRot().e1},{trailer.GetRot().e2},{trailer.GetRot().e3}\n")

    
    truck_inputs = truck_driver.GetInputs()

    
    sedan_inputs = veh.DriverInputs()
    sedan_inputs.m_throttle = 0.6
    sedan_inputs.m_steering = 0.02
    sedan_inputs.m_braking = 0.0

    
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

    step_number += 1
    realtime_timer.Spin(step_size)