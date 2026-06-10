"""
FEDA vehicle on rigid terrain with Irrlicht visualization.

Simulates a FEDA wheeled vehicle driving on a flat rigid terrain using NSC contact.
The vehicle is equipped with mesh visualization for all parts, an interactive driver,
and a chase camera that follows the vehicle. Terrain uses a custom texture.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants ===
STEPPER = 1e-3          # physics time step [s]
SIM_END = 30.0          # simulation duration [s]
RENDER_FPS = 50.0       # render frame rate [fps]
render_every = max(1, round(1.0 / (RENDER_FPS * STEPPER)))

# FEDA spawn position (flat rigid terrain at z=0)
VEH_INIT_X = 0.0
VEH_INIT_Y = 0.0
VEH_INIT_Z = 0.8        # chassis origin height above wheel contact plane

# Terrain dimensions
TERRAIN_LENGTH = 200.0
TERRAIN_WIDTH = 200.0

# === Paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# === Vehicle ===
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)
feda.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z),
    chrono.QUNIT
))
feda.SetTireType(veh.TireModelType_PAC02)
feda.SetTireStepSize(STEPPER)
feda.Initialize()

system = feda.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", feda.GetVehicle().GetMass())

# === Terrain ===
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.7, 0.7, 0.6))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FEDA - Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(feda.GetVehicle())

# Set mesh visualization for all vehicle parts
feda.GetVehicle().SetChassisVisualizationType(chrono.VisualizationType_MESH)
feda.GetVehicle().SetSuspensionVisualizationType(chrono.VisualizationType_MESH)
feda.GetVehicle().SetSteeringVisualizationType(chrono.VisualizationType_MESH)
feda.GetVehicle().SetWheelVisualizationType(chrono.VisualizationType_MESH)
feda.GetVehicle().SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Driver ===
driver = veh.ChInteractiveDriver(feda.GetVehicle())
steering_time = 1.0
throttle_time = 1.0
braking_time = 0.3
driver.SetSteeringDelta(render_every * STEPPER / steering_time)
driver.SetThrottleDelta(render_every * STEPPER / throttle_time)
driver.SetBrakingDelta(render_every * STEPPER / braking_time)
driver.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_num = 0

while vis.Run() and system.GetChTime() < SIM_END:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        sim_time = system.GetChTime()
        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        feda.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(STEPPER)
        terrain.Advance(STEPPER)
        feda.Advance(STEPPER)
        vis.Advance(STEPPER)

        step_num += 1
        realtime_timer.Spin(STEPPER)

        if system.GetChTime() >= SIM_END:
            break
