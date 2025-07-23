# Use the path follower module for the vehicle steering instead of interactive driver
import pychrono.sensor as sens

follower = veh.ChPathFollowerController(vehicle.GetVehicle(), 0.0, 0.3, 20.0, 3.0)
follower.SetLookAheadDistance(5.0)
follower.SetGains(0.02, 0.0, 0.01)

# Initialize two balls to visualize the path
sphereBall1 = chrono.ChVisualShapeSphere(0.2)
sphereBall2 = chrono.ChVisualShapeSphere(0.2)
vehicle.GetSystem().Add(sphereBall1)
vehicle.GetSystem().Add(sphereBall2)
sphereBall1.GetPos() = chrono.ChVector3d(0, 2, 0.5)
sphereBall2.GetPos() = chrono.ChVector3d(0, -2, 0.5)
vehicle.GetSystem().Update()

# Simulation loop
while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    # Update path follower controller
    follower.Update(time, )
    # render point along path
    sphereBall1.SetPos(follower.GetCurrentPos())
    sphereBall2.SetPos(follower.GetCurrentPos() + chrono.ChVector3d(0, 0.5, 0))

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)