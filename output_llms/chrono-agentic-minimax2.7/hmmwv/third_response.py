"""
HMMWV demo with custom driver system.
Vehicle: HMMWV_Full on flat rigid terrain.
Driver: Custom MyDriver class with time-based scripted inputs.
"""

import math
import os

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


class MyDriver(veh.ChDriver):
    """Custom driver with delay, throttle ramp, and sinusoidal steering."""

    def __init__(self, vehicle, delay=0.5):
        super().__init__(vehicle)
        self.delay = delay

    def Synchronize(self, time):
        if time < self.delay:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        elif time < self.delay + 0.2:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)
            self.SetSteering(0.0)
        else:
            t_ramp = time - self.delay - 0.2
            throttle = min(0.7, t_ramp / 0.2 * 0.7)
            self.SetThrottle(throttle)
            self.SetBraking(0.0)
            if time >= 2.0:
                steer = 0.3 * math.sin(0.5 * (time - 2.0))
            else:
                steer = 0.0
            self.SetSteering(steer)


# === System & paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle ===
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()
system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV - Custom Driver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

# === Custom driver ===
driver = MyDriver(hmmwv.GetVehicle(), delay=0.5)
driver.Initialize()

# === Simulation loop ===
time_step = 1e-3
sim_end = 4.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


frame = 0
step_number = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        hmmwv.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)


        step_number += 1
        realtime_timer.Spin(time_step)

        if system.GetChTime() >= sim_end:
            break
