"""
CityBus simulation with Pacejka tire model, reduced step size, and dirt terrain texture.

Changes from base configuration:
  - Tire model: TMEasy -> Pacejka 89 (PAC02)
  - Simulation step size: 1e-3 -> 5e-4
  - Terrain texture: tile4.jpg -> dirt.jpg
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Configuration ===
TIME_STEP = 5e-4          # reduced from 1e-3 for better stability
SIM_END = 6.0             # seconds
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

REC = bool(os.environ.get("SIMBENCH_RECORD"))

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create the CityBus vehicle ===
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_PAC02)          # prompt: Pacejka 89
bus.SetTireStepSize(TIME_STEP)
bus.Initialize()
system = bus.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Essential components (visible handles) ===
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === Terrain (RigidTerrain with dirt texture) ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,   # length
    200.0    # width
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - Pacejka Tire")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

# === Interactive driver ===
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / steering_time)
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / throttle_time)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / braking_time)
driver.Initialize()

# === Cache chassis reference ===
chassis = bus.GetChassisBody()  # cache: fetched once, reused every step

# === CSV logging (review-only) ===

# === Main loop ===
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
run_ok = True
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()

            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            bus.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            bus.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)


            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    run_ok = False
    raise

# === Review-only post-processing ===
