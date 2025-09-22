import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math




mesh_system = chrono.ChSystemSMC()


vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('HMMWV Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.71), 6.0, 0.5)
vis.Initialize()
vis.AttachSystem(mesh_system)


vehicle = veh.HMMWV_Full()
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-8, 0, 0.6), chrono.QUNIT))
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.Initialize()


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
terrain.Initialize(20, 20, 0.02)


driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0  
throttle_time = 1.0  
braking_time = 0.3   
driver.SetSteeringDelta(1.0 / steering_time)
driver.SetThrottleDelta(1.0 / throttle_time)
driver.SetBrakingDelta(1.0 / braking_time)






print("VEHICLE MASS: ",  vehicle.GetVehicle().GetMass())


render_steps = math.ceil(1.0 / 0.01)


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() :
    time = vehicle.GetSystem().GetChTime()

    
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    
    driver_inputs = driver.GetInputs()

    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    
    driver.Advance(0.01)
    terrain.Advance(0.01)
    vehicle.Advance(0.01)
    vis.Advance(0.01)

    
    step_number += 1

    
    realtime_timer.Spin(0.01)