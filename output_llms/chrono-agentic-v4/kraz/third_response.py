"""
Kraz truck (tractor-trailer) + BMW sedan on highway terrain.
Tracks tractor and trailer state during simulation.
Two vehicles share one system; sedan drives with fixed throttle/steering.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# review-only imports

# === Named constants ===
TIME_STEP = 1e-3
SIM_END = 20.0
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Truck initial position and orientation
TRUCK_INIT_X = 0.0
TRUCK_INIT_Y = 0.0
TRUCK_INIT_Z = 0.5
TRUCK_INIT_YAW = 0.0  # facing +X

# Sedan initial position and orientation (offset behind truck)
SEDAN_INIT_X = TRUCK_INIT_X - 8.0
SEDAN_INIT_Y = TRUCK_INIT_Y
SEDAN_INIT_Z = 0.5
SEDAN_INIT_YAW = 0.0

# Sedan control inputs (fixed throttle and steering)
SEDAN_THROTTLE = 0.5
SEDAN_STEERING = 0.0
SEDAN_BRAKING = 0.0

# === Data paths ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Create Kraz truck (tractor-trailer) ===
print("Creating Kraz truck...")
kraz = veh.Kraz()
kraz.SetContactMethod(chrono.ChContactMethod_NSC)
kraz.SetChassisFixed(False)
kraz.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(TRUCK_INIT_X, TRUCK_INIT_Y, TRUCK_INIT_Z),
        chrono.QuatFromAngleY(TRUCK_INIT_YAW),
    )
)
kraz.SetTireStepSize(TIME_STEP)
kraz.Initialize()
system = kraz.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
print("VEHICLE MASS: ", kraz.GetTractor().GetMass())

tractor = kraz.GetTractor()

# Cache key truck bodies for state logging
tractor_body = kraz.GetTractorChassisBody()
trailer_chassis = kraz.GetTrailer().GetChassis()

# === Create BMW sedan on the SAME system ===
print("Creating BMW sedan...")
sedan = veh.BMW_E90(system)  # share system with Kraz
sedan.SetContactMethod(chrono.ChContactMethod_NSC)
sedan.SetChassisFixed(False)
sedan.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(SEDAN_INIT_X, SEDAN_INIT_Y, SEDAN_INIT_Z),
        chrono.QuatFromAngleY(SEDAN_INIT_YAW),
    )
)
sedan.SetTireType(veh.TireModelType_RIGID)
sedan.SetTireStepSize(TIME_STEP)
sedan.Initialize()
print("SEDAN MASS: ", sedan.GetVehicle().GetMass())

# === Flat highway terrain ===
print("Creating highway terrain...")
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.01)

# Large flat highway patch
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    200.0,   # length X
    20.0,    # width Y
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 4)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Truck + BMW Sedan")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 3.0), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(kraz.GetTractor())

# === Drivers ===
driver_kraz = veh.ChInteractiveDriverIRR(vis)
driver_kraz.SetSteeringDelta(TIME_STEP / 1.0)
driver_kraz.SetThrottleDelta(TIME_STEP / 1.0)
driver_kraz.SetBrakingDelta(TIME_STEP / 0.3)
driver_kraz.Initialize()

# Sedan driver — fixed throttle/steering (ChDriver subclass)
class SedanDriver(veh.ChDriver):
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(SEDAN_THROTTLE)
        self.SetSteering(SEDAN_STEERING)
        self.SetBraking(SEDAN_BRAKING)

driver_sedan = SedanDriver(sedan.GetVehicle())
driver_sedan.Initialize()

# === Review-only: recording setup ===

# === Main simulation loop ===
print("Starting simulation...")
frame = 0
step_number = 0

# CSV for state logging (tractor + trailer)

while vis.Run() and system.GetChTime() < SIM_END:
    # Throttled rendering
    if step_number % RENDER_EVERY == 0:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    kraz_inputs = driver_kraz.GetInputs()
    sedan_inputs = driver_sedan.GetInputs()

    # Synchronize
    time = system.GetChTime()
    driver_kraz.Synchronize(time)
    driver_sedan.Synchronize(time)
    terrain.Synchronize(time)
    kraz.Synchronize(time, kraz_inputs, terrain)
    sedan.Synchronize(time, sedan_inputs, terrain)
    vis.Synchronize(time, kraz_inputs)

    # Advance
    driver_kraz.Advance(TIME_STEP)
    driver_sedan.Advance(TIME_STEP)
    terrain.Advance(TIME_STEP)
    kraz.Advance(TIME_STEP)
    sedan.Advance(TIME_STEP)
    vis.Advance(TIME_STEP)

    # Log state (tractor + trailer + sedan)

    step_number += 1

    if system.GetChTime() >= SIM_END:
        break

# === Review-only post-processing ===

print("Simulation complete.")
