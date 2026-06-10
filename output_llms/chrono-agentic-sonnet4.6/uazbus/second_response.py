"""
UAZBUS double lane change simulation on rigid terrain with concrete texture.

System: ChSystemNSC (NSC contact method), wrapper-managed UAZBUS vehicle.
Bodies: UAZBUS chassis + 4 wheels/tires; rigid terrain patch.
Driver: scripted time-based double lane change — custom ChDriver subclass
        sets steering/throttle/braking by time phase to simulate a double lane
        change maneuver and then braking.
Expected behavior: vehicle starts at (-40, 0, 0.5), accelerates, steers left
to change lane, steers back to center, then right to return, then brakes.
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants ===
time_step = 1e-3           # physics step (s)
sim_end = 30.0             # total simulation duration (s)
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

TERRAIN_LENGTH = 300.0
TERRAIN_WIDTH = 60.0
INIT_POS = chrono.ChVector3d(-40, 0, 0.5)
INIT_ROT = chrono.QuatFromAngleZ(0)

# === Data paths (truth-required for all catalog vehicle demos) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Vehicle setup ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()         # main chassis rigid body  # cache: fetched once, reused every step
# wheels/spindles: vehicle.GetAxle(i)... ; terrain: RigidTerrain patch body below
# joints: suspension + steering links created inside the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Visualization types — after Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === Terrain ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Driver — scripted double lane change (scored core: time-based inputs) ===
class DoubleManeuverDriver(veh.ChDriver):
    """Scripted time-based driver for double lane change and brake maneuver."""
    def __init__(self, veh_obj):
        super().__init__(veh_obj)

    def Synchronize(self, time):
        # Phase 0: 0..1.5 s   — idle
        if time < 1.5:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        # Phase 1: 1.5..2.5 s — accelerate
        elif time < 2.5:
            self.SetThrottle(0.5)
            self.SetSteering(0.0)
            self.SetBraking(0.0)
        # Phase 2: 2.5..4.0 s — steer left (first lane change)
        elif time < 4.0:
            self.SetThrottle(0.5)
            self.SetSteering(0.5)
            self.SetBraking(0.0)
        # Phase 3: 4.0..5.5 s — steer back toward center
        elif time < 5.5:
            self.SetThrottle(0.4)
            self.SetSteering(-0.2)
            self.SetBraking(0.0)
        # Phase 4: 5.5..7.0 s — steer right (return lane change)
        elif time < 7.0:
            self.SetThrottle(0.4)
            self.SetSteering(-0.5)
            self.SetBraking(0.0)
        # Phase 5: 7.0..8.5 s — steer back to center
        elif time < 8.5:
            self.SetThrottle(0.3)
            self.SetSteering(0.2)
            self.SetBraking(0.0)
        # Phase 6: 8.5+ s — brake to stop
        else:
            self.SetThrottle(0.0)
            self.SetSteering(0.0)
            self.SetBraking(0.8)

driver = DoubleManeuverDriver(vehicle.GetVehicle())
driver.Initialize()

# === Visualization ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change — Concrete Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(vehicle.GetVehicle())

# === Review-only setup ===


realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0

# === Main loop ===
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        sim_time = sys.GetChTime()

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)


        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)

        if sys.GetChTime() >= sim_end:
            break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass  # cleanup handled below

# === Post-processing ===
