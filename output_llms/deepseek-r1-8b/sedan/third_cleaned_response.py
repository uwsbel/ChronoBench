import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math


initLoc = chrono.ChVector3d(0, 0, 1)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)


steering_time = 5.0  
throttle_time = 1.0  
braking_time = 0.3   
driver_max_speed = 50.0  


step_size = 1e-4  
render_step_size = 1.0 / 50  


speed_ref = 50.0  


PID = veh.ChPIDParams()
PID.SetPID(PID.PID_GAMMA, PID.PID_KP, PID.PID_KI)  
pid = veh.ChPIDController(PID)


vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(step_size)

vehicle.Initialize()


vehicle.GetVehicle().SetPIDController(pid, "throttle")


terrain = veh.RigidTerrain(vehicle.GetSystem())
terrainHeight = 0      
terrainLength = 100.0  
terrainWidth = 100.0   


highway_mesh = terrain.AddHighwayMesh(
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth,
    "highway/straight/straight_road_1 texture texture.png"
)
highway_mesh.SetMaterial(chrono.ChContactMaterialNSC())
highway_mesh.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Sedan')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())


driver = veh.ChInteractiveDriverIRR(vis)


driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)

driver.Initialize()


driver.SetReferenceSpeed(speed_ref)


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    current_speed = vehicle.GetVehicle().GetSpeed()
    speed_error = speed_ref - current_speed

    
    pid.Update(speed_error)
    throttle_input = pid.Output()

    
    throttle_input = max(0, min(1, throttle_input))

    
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    
    driver.SetThrottleInput(throttle_input)

    
    step_number += 1

    
    realtime_timer.Spin(step_size)