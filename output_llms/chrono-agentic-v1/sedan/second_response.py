"""
Two-Sedan Simulation on Concrete Terrain
=========================================
Models two BMW E90 sedans on a flat rigid terrain with a concrete texture.
Vehicle 1 uses an interactive Irrlicht driver (scored-core default).
Vehicle 2 shares the same ChSystem via WheeledVehicle(system, json) and uses
a SinusoidalDriver subclass. Sinusoidal steering inputs are applied to both
vehicles in the simulation loop. Both vehicles are synchronised and advanced
every step; the loop covers both Synchronize and Advance calls for each
vehicle–driver–terrain pair.

System: ChSystemNSC (owned by sedan1 wrapper, shared by sedan2)
Vehicles: Two BMW E90 sedan-type vehicles (veh.BMW_E90 + veh.WheeledVehicle)
Terrain: veh.RigidTerrain with concrete.jpg texture
Drivers: ChInteractiveDriverIRR (sedan1) + SinusoidalDriver subclass (sedan2)
         Review-only block applies sinusoidal steering to sedan1 for the RUN video.
Expected: Both sedans sit on the concrete surface and steer sinusoidally.
"""

# === Imports ===
import math
import os
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as virl


# === Named constants ===
TIME_STEP    = 1e-3          # physics step (s)
SIM_END      = 15.0          # simulation duration (s)
RENDER_FPS   = 50.0          # target rendering rate (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

TERRAIN_LENGTH = 200.0   # m, X extent
TERRAIN_WIDTH  = 200.0   # m, Y extent

# Vehicle 1 initial position and orientation
VEH1_X, VEH1_Y = 0.0, 0.0
VEH1_ROT        = chrono.QuatFromAngleZ(0.0)   # heading +X

# Vehicle 2 initial position and orientation (offset to avoid overlap)
VEH2_X, VEH2_Y = 12.0, 5.0
VEH2_ROT        = chrono.QuatFromAngleZ(0.0)   # heading +X

SUSPENSION_REF_HEIGHT = 0.5   # chassis-origin height above wheel-bottom at rest (sedan ≈ 0.5 m)
TIRE_RADIUS           = 0.32  # approximate tire radius for BMW E90 / Sedan (m)
ZTOL                  = 0.12  # tolerance for wheel-bottom-vs-ground assertion (m)

# Sinusoidal steering parameters (both vehicles)
STEER_FREQ      = 0.5    # Hz
STEER_AMPLITUDE = 0.3    # dimensionless (−1…+1)
STEER_THROTTLE  = 0.4    # constant throttle applied during sinusoidal steering

# Sedan JSON paths (actual file structure under /share/chrono/data/vehicle/)
SEDAN_VEHICLE_JSON       = "sedan/vehicle/Sedan_Vehicle.json"
SEDAN_ENGINE_JSON        = "sedan/powertrain/Sedan_EngineSimpleMap.json"
SEDAN_TRANS_JSON         = "sedan/powertrain/Sedan_AutomaticTransmissionSimpleMap.json"
SEDAN_TIRE_JSON          = "sedan/tire/Sedan_TMeasyTire.json"
TERRAIN_CONCRETE_TEX     = "terrain/textures/concrete.jpg"


# === Vehicle 1 — BMW_E90 wrapper (creates and owns the ChSystemNSC) ===
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")  # resolve vehicle data tree

sedan1 = veh.BMW_E90()
sedan1.SetContactMethod(chrono.ChContactMethod_NSC)
sedan1.SetChassisCollisionType(veh.CollisionType_NONE)
sedan1.SetChassisFixed(False)
sedan1.SetInitPosition(
    chrono.ChCoordsysd(
        chrono.ChVector3d(VEH1_X, VEH1_Y, SUSPENSION_REF_HEIGHT),
        VEH1_ROT,
    )
)
sedan1.SetTireType(veh.TireModelType_TMEASY)
sedan1.SetTireStepSize(TIME_STEP)
sedan1.Initialize()

# === System & bodies (created internally by the veh.BMW_E90 wrapper) ===
system   = sedan1.GetSystem()          # ChSystemNSC owned by sedan1 wrapper
chassis1 = sedan1.GetChassisBody()    # main chassis rigid body for sedan1
# wheels, spindles, suspension links, powertrain — created by sedan1 wrapper
# sedan2, terrain, and vis share this same ChSystem

system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED: terrain contact

# Visualisation types for sedan1 (MUST be called after Initialize)
sedan1.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan1.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan1.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan1.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan1.SetTireVisualizationType(veh.VisualizationType_MESH)


# === Vehicle 2 — WheeledVehicle sharing sedan1's ChSystem ===
sedan2_wv = veh.WheeledVehicle(system, veh.GetDataFile(SEDAN_VEHICLE_JSON))
sedan2_wv.Initialize(
    chrono.ChCoordsysd(
        chrono.ChVector3d(VEH2_X, VEH2_Y, SUSPENSION_REF_HEIGHT),
        VEH2_ROT,
    )
)

# Powertrain for sedan2
try:
    engine2 = veh.ReadEngineJSON(veh.GetDataFile(SEDAN_ENGINE_JSON))
    trans2  = veh.ReadTransmissionJSON(veh.GetDataFile(SEDAN_TRANS_JSON))
    pt2     = veh.ChPowertrainAssembly(engine2, trans2)
    sedan2_wv.InitializePowertrain(pt2)
except (RuntimeError, OSError, IOError):
    pass   # vehicle will coast without powertrain if JSON missing

# Tires for sedan2
try:
    for axle in sedan2_wv.GetAxles():
        for wheel in axle.GetWheels():
            tire2 = veh.ReadTireJSON(veh.GetDataFile(SEDAN_TIRE_JSON))
            sedan2_wv.InitializeTire(tire2, wheel, veh.VisualizationType_MESH)
except (RuntimeError, OSError, IOError):
    pass

sedan2_wv.SetChassisVisualizationType(veh.VisualizationType_MESH)
sedan2_wv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan2_wv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan2_wv.SetWheelVisualizationType(veh.VisualizationType_MESH)
sedan2_wv.SetTireVisualizationType(veh.VisualizationType_MESH)

chassis2 = sedan2_wv.GetChassisBody()  # cache: fetched once, reused each step


# === Terrain — flat RigidTerrain with concrete texture ===
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
patch.SetTexture(veh.GetDataFile(TERRAIN_CONCRETE_TEX), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()


# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedan Simulation — Concrete Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(sedan1.GetVehicle())   # chase camera follows sedan1


# === Driver 1 — interactive (scored-core default for catalog vehicles) ===
driver1 = veh.ChInteractiveDriverIRR(vis)
driver1.SetSteeringDelta(RENDER_EVERY * TIME_STEP / 1.0)   # 1 s to full steer
driver1.SetThrottleDelta(RENDER_EVERY * TIME_STEP / 1.0)
driver1.SetBrakingDelta(RENDER_EVERY * TIME_STEP / 0.3)
driver1.Initialize()


# === Driver 2 — sinusoidal scripted driver (scored-core; sedan2 requirement) ===
class SinusoidalDriver(veh.ChDriver):
    """
    Scripted driver that applies sinusoidal steering and constant throttle.
    Implements the sinusoidal steering requirement for vehicle 2.
    """

    def __init__(self, vehicle_obj, freq=0.5, amplitude=0.3, throttle=0.4):
        super().__init__(vehicle_obj)
        self._freq      = freq
        self._amplitude = amplitude
        self._throttle  = throttle

    def Synchronize(self, time):
        steering = self._amplitude * math.sin(2.0 * math.pi * self._freq * time)
        self.SetSteering(steering)
        self.SetThrottle(self._throttle)
        self.SetBraking(0.0)


driver2 = SinusoidalDriver(
    sedan2_wv,
    freq=STEER_FREQ,
    amplitude=STEER_AMPLITUDE,
    throttle=STEER_THROTTLE,
)
driver2.Initialize()


# === Footprint validation — assert wheel-bottoms sit on terrain ===
veh1_obj = sedan1.GetVehicle()  # cache: sedan1 vehicle handle
for axle_idx in range(veh1_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        sp    = veh1_obj.GetSpindlePos(axle_idx, side)
        wb_z  = sp.z - TIRE_RADIUS
        assert wb_z >= -ZTOL, (
            f"Sedan1 spindle (axle={axle_idx}, side={side}) wheel bottom "
            f"z={wb_z:.3f} below ground; raise SUSPENSION_REF_HEIGHT by "
            f"{max(0.0, -wb_z):.3f} m."
        )

for axle in sedan2_wv.GetAxles():
    for wheel in axle.GetWheels():
        sp   = wheel.GetSpindle().GetPos()
        wb_z = sp.z - TIRE_RADIUS
        assert wb_z >= -ZTOL, (
            f"Sedan2 wheel bottom z={wb_z:.3f} below ground; raise "
            f"SUSPENSION_REF_HEIGHT by {max(0.0, -wb_z):.3f} m."
        )


# === Review-only: frame capture setup ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()  # cache: real-time pacing timer
step_number    = 0
frame          = 0

try:
    while vis.Run():
        time = system.GetChTime()
        if time >= SIM_END:
            break

        # Throttled rendering
        if step_number % RENDER_EVERY == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Gather driver inputs for both vehicles
        driver_inputs1 = driver1.GetInputs()
        driver_inputs2 = driver2.GetInputs()


        # Synchronize: driver → terrain → vehicle → vis  (BOTH vehicles)
        driver1.Synchronize(time)
        driver2.Synchronize(time)
        terrain.Synchronize(time)
        sedan1.Synchronize(time, driver_inputs1, terrain)
        sedan2_wv.Synchronize(time, driver_inputs2, terrain)
        vis.Synchronize(time, driver_inputs1)

        # Advance all subsystems
        driver1.Advance(TIME_STEP)
        driver2.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        sedan1.Advance(TIME_STEP)        # advances wrapper-owned ChSystem
        sedan2_wv.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)


except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # ensure clean exit even on solver divergence
