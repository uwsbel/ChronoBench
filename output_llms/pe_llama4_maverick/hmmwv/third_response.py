class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay
        self.time = 0

    def Synchronize(self, time):
        self.time = time
        inputs = veh.Inputs()
        if time > self.delay + 0.2:
            inputs.m_throttle = min((time - self.delay - 0.2) / 1.0, 0.7)  # Gradually increase throttle to 0.7
        else:
            inputs.m_throttle = 0
        if time > self.delay + 2.0:
            inputs.m_steering = 0.5 * math.sin(2 * math.pi * (time - self.delay - 2.0))  # Sinusoidal steering
        else:
            inputs.m_steering = 0
        inputs.m_braking = 0
        self.SetInputs(inputs)

driver = MyDriver(vehicle.GetVehicle(), 0.5)

while vis.Run() and vehicle.GetSystem().GetChTime() < 4.0:
    # Simulation loop content remains the same

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math

# ... (other initializations remain the same)

class MyDriver(veh.ChDriver):
    def __init__(self, vehicle, delay):
        super().__init__(vehicle)
        self.delay = delay
        self.time = 0

    def Synchronize(self, time):
        self.time = time
        inputs = veh.Inputs()
        if time > self.delay + 0.2:
            inputs.m_throttle = min((time - self.delay - 0.2) / 1.0, 0.7)  
        else:
            inputs.m_throttle = 0
        if time > self.delay + 2.0:
            inputs.m_steering = 0.5 * math.sin(2 * math.pi * (time - self.delay - 2.0))  
        else:
            inputs.m_steering = 0
        inputs.m_braking = 0
        self.SetInputs(inputs)

# Create the HMMWV vehicle, set parameters, and initialize
vehicle = veh.HMMWV_Full()
# ... (vehicle settings remain the same)

# Create the terrain
# ... (terrain setup remains the same)

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
# ... (visualization settings remain the same)

# Create the custom driver system
driver = MyDriver(vehicle.GetVehicle(), 0.5)

# Simulation loop with end condition
render_steps = math.ceil(render_step_size / step_size)
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run() and vehicle.GetSystem().GetChTime() < 4.0:
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    vis.Synchronize(time, driver.GetInputs())

    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)