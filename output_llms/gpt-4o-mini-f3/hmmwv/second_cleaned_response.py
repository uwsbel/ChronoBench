import pychrono.sensor as sens

follower = veh.ChPathFollowerController(vehicle.GetVehicle(), 0.0, 0.3, 20.0, 3.0)
follower.SetLookAheadDistance(5.0)
follower.SetGains(0.02, 0.0, 0.01)


sphereBall1 = chrono.ChVisualShapeSphere(0.2)
sphereBall2 = chrono.ChVisualShapeSphere(0.2)
vehicle.GetSystem().Add(sphereBall1)
vehicle.GetSystem().Add(sphereBall2)
sphereBall1.GetPos() = chrono.ChVector3d(0, 2, 0.5)
sphereBall2.GetPos() = chrono.ChVector3d(0, -2, 0.5)
vehicle.GetSystem().Update()


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    
    follower.Update(time, )
    
    sphereBall1.SetPos(follower.GetCurrentPos())
    sphereBall2.SetPos(follower.GetCurrentPos() + chrono.ChVector3d(0, 0.5, 0))

    
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