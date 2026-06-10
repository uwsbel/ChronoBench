import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math






initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


vis_type = veh.VisualizationType_MESH


chassis_collision_type = veh.CollisionType_NONE


tire_model = veh.TireModelType_TMEASY


terrainHeight = 0
terrainLength = 200.0
terrainWidth = 100.0


trackPoint = chrono.ChVector3d(-10.0, 0.0, 2.8)


step_size = 1e-3
tire_step_size = step_size


render_step_size = 1.0 / 50  





vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)

vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type, vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type, vis_type)
vehicle.SetWheelVisualizationType(vis_type, vis_type)
vehicle.SetTireVisualizationType(vis_type, vis_type)





terrain = veh.RigidTerrain(vehicle.GetSystem())

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

terrain.Initialize()





driver = veh.ChInteractiveDriverIRR(vis := chronoirr.ChVisualSystemIrrlicht())


driver = veh.ChDriver(vehicle.GetTractor())


driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0.0
driver_inputs.m_throttle = 0.0
driver_inputs.m_braking = 0.0





vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(trackPoint, 12.0, 0.5)
vis.SetChaseCameraState(veh.ChaseCamera.Track)
vis.SetChaseCameraMultipliers(1e-4, 10)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetTractor())


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0   
throttle_time = 1.0   
braking_time = 0.3    

driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()






render_steps = math.ceil(render_step_size / step_size)


step_number = 0
render_frame = 0

vehicle.GetSystem().SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

realtime_timer = chrono.ChRealtimeStepTimer()

print("Starting Kraz vehicle simulation...")
print("Use arrow keys to control the vehicle:")
print("  UP/DOWN  - Throttle/Brake")
print("  LEFT/RIGHT - Steering")

while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    if step_number % render_steps == 0:
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

print("Simulation complete.")