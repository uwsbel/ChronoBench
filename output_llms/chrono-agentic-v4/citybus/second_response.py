"""
CityBus simulation with data-driven driver system.

plan_type: mbs_in_scene
vehicle: CityBus with ChDataDriver (programmatic throttle/steering/brake schedule)
terrain: RigidTerrain (flat NSC)
objective: demonstrate ChDataDriver with a piecewise driver-input schedule
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# -------------------------------------------------------------------------
# 1. Named constants
# -------------------------------------------------------------------------
STEP_SIZE = 5e-3          # physics time-step (s)
SIM_END   = 12.0          # total simulation duration (s)
RENDER_FPS = 50.0          # display/video frame rate
render_every = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))

# -------------------------------------------------------------------------
# 2. System + gravity
# -------------------------------------------------------------------------
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetVehicleDataPath(chrono.GetChronoDataPath() + "vehicle/")

bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
bus.SetTireType(veh.TireModelType_TMEASY)
bus.SetTireStepSize(STEP_SIZE)
bus.Initialize()

sys = bus.GetSystem()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())

# === System & bodies (created by the veh.CityBus wrapper) ===
# sys          : ChSystemNSC owned by the wrapper
# chassis      : main bus body
# wheels       : via bus.GetVehicle().GetAxles()
# terrain      : RigidTerrain patch below
# driver       : ChDataDriver (data-driven schedule)
# vis          : ChWheeledVehicleVisualSystemIrrlicht

# -------------------------------------------------------------------------
# 3. Terrain — RigidTerrain with NSC patch
# -------------------------------------------------------------------------
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    200.0,   # length (m)
    200.0,   # width (m)
)
terrain.Initialize()
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)

# -------------------------------------------------------------------------
# 4. Driver — ChDataDriver with piecewise schedule
#    At 0.0s: throttle=0.0, steering=0.0, braking=0.0
#    At 0.1s: throttle=1.0, steering=0.0, braking=0.0
#    At 0.5s: throttle=1.0, steering=0.7, braking=0.0
# -------------------------------------------------------------------------
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)
driver.Initialize()

# -------------------------------------------------------------------------
# 5. Visualization — Irrlicht
# -------------------------------------------------------------------------
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus — ChDataDriver")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.5), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(bus.GetVehicle())

# -------------------------------------------------------------------------
# 6. Main loop — Synchronize/Advance order: driver -> terrain -> vehicle -> vis
# -------------------------------------------------------------------------
frame = 0
realtime_timer = chrono.ChRealtimeStepTimer()
sim_time = sys.GetChTime()

while vis.Run() and sim_time < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = sys.GetChTime()

        driver_inputs = driver.GetInputs()
        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        bus.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        bus.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)

        if sim_time >= SIM_END:
            break

    frame += 1
    realtime_timer.Spin(STEP_SIZE)
    sim_time = sys.GetChTime()
