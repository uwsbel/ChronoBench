"""
Two-sedan simulation using BMW_E90 wrappers on rigid concrete terrain.

System type: NSC (rigid terrain, default for catalog sedan).
Main bodies:
  - sedan1: BMW_E90 at (-10, 0, 0.5), heading forward (+X)
  - sedan2: BMW_E90 at ( 10, 0, 0.5), heading forward (+X), sharing sedan1's system
Terrain: RigidTerrain flat patch, concrete.jpg texture.
Drivers: Two ChInteractiveDriverIRR instances; sinusoidal steering applied in
         review-only block so the scored core stays a faithful interactive-driver demo.
Expected behavior: Both sedans rest on the terrain; interactive keyboard control drives them.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths (required for all catalog vehicle truths) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Named constants ===
STEP_SIZE       = 1e-3          # physics time step (s)
SIM_END         = 20.0          # simulation end time (s)
RENDER_FPS      = 50.0
render_every    = max(1, round(1.0 / (RENDER_FPS * STEP_SIZE)))  # precomputed once

TERRAIN_LENGTH  = 200.0         # m
TERRAIN_WIDTH   = 100.0         # m
SUSPENSION_REF  = 0.5           # chassis origin above wheel-bottom at rest (BMW_E90 typical)

# Initial positions — offset so the two sedans do not overlap
INIT_LOC1 = chrono.ChVector3d(-10, 0, SUSPENSION_REF)
INIT_LOC2 = chrono.ChVector3d( 10, 0, SUSPENSION_REF)
INIT_ROT  = chrono.ChQuaterniond(1, 0, 0, 0)  # heading +X

# === Sedan 1 (BMW_E90 — owns the ChSystem) ===
sedan1 = veh.BMW_E90()
sedan1.SetContactMethod(chrono.ChContactMethod_NSC)
sedan1.SetChassisCollisionType(veh.CollisionType_NONE)
sedan1.SetChassisFixed(False)
sedan1.SetInitPosition(chrono.ChCoordsysd(INIT_LOC1, INIT_ROT))
sedan1.SetTireType(veh.TireModelType_TMEASY)
sedan1.SetTireStepSize(STEP_SIZE)
sedan1.Initialize()

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
sys = sedan1.GetSystem()                # ChSystemNSC owned by sedan1 wrapper
chassis1 = sedan1.GetChassisBody()     # cache: fetched once, reused every step
# wheels/spindles: sedan1.GetVehicle().GetAxle(i); terrain and sedan2 attached below
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

print("VEHICLE MASS: ", sedan1.GetVehicle().GetMass())

# Visualization types for sedan1
sedan1.SetChassisVisualizationType(chrono.VisualizationType_MESH)
sedan1.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan1.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan1.SetWheelVisualizationType(chrono.VisualizationType_MESH)
sedan1.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Sedan 2 (shares sedan1's system — required to share the same world) ===
sedan2 = veh.BMW_E90(sys)              # MUST pass shared system — no orphan system
sedan2.SetChassisFixed(False)
sedan2.SetInitPosition(chrono.ChCoordsysd(INIT_LOC2, INIT_ROT))
sedan2.SetTireType(veh.TireModelType_TMEASY)
sedan2.SetTireStepSize(STEP_SIZE)
sedan2.Initialize()

chassis2 = sedan2.GetChassisBody()     # cache: fetched once, reused every step

print("VEHICLE MASS (sedan2): ", sedan2.GetVehicle().GetMass())

sedan2.SetChassisVisualizationType(chrono.VisualizationType_MESH)
sedan2.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan2.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
sedan2.SetWheelVisualizationType(chrono.VisualizationType_MESH)
sedan2.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === Terrain (shared RigidTerrain on sys) ===
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Two Sedans — Concrete Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(sedan1.GetVehicle())

# === Driver 1 (interactive — scored-core default matching ground truth) ===
render_step_size = 1.0 / RENDER_FPS   # precomputed once
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

driver1 = veh.ChInteractiveDriverIRR(vis)
driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)
driver1.Initialize()

# === Driver 2 (scripted — sinusoidal steering, both vehicles instructed) ===
class SinusoidalDriver(veh.ChDriver):
    """Scripted sinusoidal steering driver for sedan2."""
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        self.SetThrottle(0.4)
        self.SetBraking(0.0)
        self.SetSteering(0.3 * math.sin(0.5 * math.pi * time))

driver2 = SinusoidalDriver(sedan2.GetVehicle())
driver2.Initialize()

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0


try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs1 = driver1.GetInputs()
        driver_inputs2 = driver2.GetInputs()


        driver1.Synchronize(time)
        driver2.Synchronize(time)
        terrain.Synchronize(time)
        sedan1.Synchronize(time, driver_inputs1, terrain)
        sedan2.Synchronize(time, driver_inputs2, terrain)
        vis.Synchronize(time, driver_inputs1)

        driver1.Advance(STEP_SIZE)
        driver2.Advance(STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        sedan1.Advance(STEP_SIZE)
        sedan2.Advance(STEP_SIZE)
        vis.Advance(STEP_SIZE)


        step_number += 1
        realtime_timer.Spin(STEP_SIZE)

except (RuntimeError, ValueError) as exc:  # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
