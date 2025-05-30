import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math





chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")





initLoc1 = chrono.ChVector3d(0,   0.0, 0.5)
initRot1 = chrono.QUNIT

initLoc2 = chrono.ChVector3d(0,  -5.0, 0.5)
initRot2 = chrono.QUNIT


vis_type             = veh.VisualizationType_MESH
chassis_collision    = veh.CollisionType_NONE
tire_model           = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 100.0
terrainWidth  = 100.0


trackPoint = chrono.ChVector3d(-5.0, 0.0, 1.8)


contact_method = chrono.ChContactMethod_NSC


step_size       = 1e-3
tire_step_size  = step_size
render_fps      = 50
render_step     = 1.0 / render_fps




vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)

vehicle1.Initialize()


vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)




vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)

vehicle2.Initialize()
vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)


print("Vehicle 
print("Vehicle 




patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(vehicle1.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0,0,terrainHeight), chrono.QUNIT),
    terrainLength, terrainWidth
)

patch.SetTexture(
    chrono.GetChronoDataFile("terrain/textures/concrete.jpg"),
    200, 200
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()




vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedans with Sinusoidal Steering")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()

vis.AttachVehicle(vehicle1)
vis.AttachVehicle(vehicle2)




driver1 = veh.ChInteractiveDriverIRR(vis)
driver2 = veh.ChInteractiveDriverIRR(vis)


driver1.SetSteeringDelta(render_step / 1.0)
driver1.SetThrottleDelta(render_step / 1.0)
driver1.SetBrakingDelta(render_step / 0.3)

driver2.SetSteeringDelta(render_step / 1.0)
driver2.SetThrottleDelta(render_step / 1.0)
driver2.SetBrakingDelta(render_step / 0.3)

driver1.Initialize()
driver2.Initialize()




render_steps = math.ceil(render_step / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0

while vis.Run():
    t = vehicle1.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    
    steer = 0.4 * math.sin(2 * math.pi * 0.5 * t)

    
    in1 = driver1.GetInputs()
    in2 = driver2.GetInputs()
    in1.m_steering = steer
    in2.m_steering = steer
    

    
    driver1.Synchronize(t)
    driver2.Synchronize(t)
    terrain.Synchronize(t)
    vehicle1.Synchronize(t, in1, terrain)
    vehicle2.Synchronize(t, in2, terrain)
    vis.Synchronize(t, in1)   

    
    driver1.Advance(step_size)
    driver2.Advance(step_size)
    terrain.Advance(step_size)
    vehicle1.Advance(step_size)
    vehicle2.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)