import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np
import pychrono.sensor as sensor

# ... (rest of the script remains the same until this point)

# Initialize sensor manager
sensor_manager = sensor.ChSensorManager()
sensor_manager.Initialize(vehicle.GetSystem())

# Create and add random boxes to the simulation
np.random.seed(0)
num_boxes = 10
box_positions = np.random.rand(num_boxes, 3) * 10 - 5
box_orientations = np.random.rand(num_boxes, 3)
for i in range(num_boxes):
    box = chrono.ChBody()
    box.SetPos(chrono.ChVector3d(box_positions[i, 0], box_positions[i, 1], box_positions[i, 2]))
    box.SetOrientation(chrono.ChQuaterniond(box_orientations[i, 0], box_orientations[i, 1], box_orientations[i, 2], 0))
    box_shape = chrono.ChVisualShapeBox(1, 1, 1)
    box.AddVisualShape(box_shape)
    vehicle.GetSystem().Add(box)

# Change terrain texture
terrain.GetPatch(0).SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)

# Update sensor manager within the simulation loop
while vis.Run() :
    time = vehicle.GetSystem().GetChTime()
    # Render scene and output POV-Ray data
    if (step_number % render_steps == 0) :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1
    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    sensor_manager.Synchronize(time)  # Update sensor manager

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)
    sensor_manager.Advance(step_size)  # Update sensor manager

    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)