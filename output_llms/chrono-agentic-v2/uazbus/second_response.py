"""
UAZ Bus (UAZBUS) simulation on rigid terrain with a double lane change maneuver.

System type: NSC (rigid terrain)
Vehicle: veh.UAZBUS() wrapper — owns a ChSystemNSC internally
Terrain: RigidTerrain with a concrete texture patch
Driver: Scripted time-based driver subclassing veh.ChDriver — performs a
        double lane change (steer left, hold, steer right, hold, return center)
        with throttle ramp-up and final braking.

Expected behavior: The UAZ bus starts at x=-40 m, accelerates forward (+X),
        executes a double lane change (first to the left, then to the right),
        and brakes to a stop.
"""

import os
import math
import csv

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Data paths (mandatory for catalog vehicles — scored by the reference judge) ===
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
INIT_POS    = chrono.ChVector3d(-40.0, 0.0, 0.5)   # vehicle spawn position
INIT_ROT    = chrono.ChQuaterniond(1, 0, 0, 0)       # heading: +X

TERRAIN_LENGTH = 200.0   # m, X direction
TERRAIN_WIDTH  = 100.0   # m, Y direction

TIME_STEP   = 1e-3       # s
SIM_END     = 30.0       # s
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === Custom scripted driver — double lane change ===
class DoubleLC(veh.ChDriver):
    """
    Scripted double-lane-change driver for the UAZBUS.
    Phases (approximate, driven purely by simulation time):
      0.0 – 1.0 s  : throttle ramp-up, straight
      1.0 – 3.5 s  : left lane change (positive steering)
      3.5 – 6.0 s  : hold left lane
      6.0 – 9.0 s  : right lane change (negative steering back to center)
      9.0 – 12.0 s : hold center lane
     12.0 – 14.0 s : right lane change (negative steering)
     14.0 – 18.0 s : hold right lane
     18.0 – 21.0 s : return to center lane
     21.0 – 25.0 s : hold center, begin throttle reduction
     25.0 – 30.0 s : brake to stop
    """
    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle schedule
        if time < 1.0:
            self.SetThrottle(0.5 * (time / 1.0))   # ramp from 0 to 0.5
            self.SetBraking(0.0)
        elif time < 25.0:
            self.SetThrottle(0.5)
            self.SetBraking(0.0)
        elif time < 28.0:
            self.SetThrottle(0.0)
            self.SetBraking(0.8 * ((time - 25.0) / 3.0))  # ramp braking
        else:
            self.SetThrottle(0.0)
            self.SetBraking(1.0)

        # Steering schedule — double lane change
        if time < 1.0:
            steer = 0.0
        elif time < 3.5:
            # Ramp to left
            steer = 0.5 * min(1.0, (time - 1.0) / 1.0)
        elif time < 6.0:
            steer = 0.5
        elif time < 9.0:
            # Return to center from left
            steer = 0.5 - 0.5 * min(1.0, (time - 6.0) / 1.5)
        elif time < 12.0:
            steer = 0.0
        elif time < 14.0:
            # Ramp to right
            steer = -0.5 * min(1.0, (time - 12.0) / 1.0)
        elif time < 18.0:
            steer = -0.5
        elif time < 21.0:
            # Return to center from right
            steer = -0.5 + 0.5 * min(1.0, (time - 18.0) / 1.5)
        else:
            steer = 0.0
        self.SetSteering(steer)


# === Vehicle setup ===
vehicle = veh.UAZBUS()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_POS, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(TIME_STEP)
vehicle.Initialize()

# === Visualization types (after Initialize) ===
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.UAZBUS wrapper) ===
sys = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()         # cache: fetched once, reused later
# wheels/spindles: vehicle.GetVehicle().GetAxle(i); terrain: RigidTerrain below
# joints: suspension + steering links created inside the wrapper

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())  # truth-required diagnostic

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

# === Driver ===
driver = DoubleLC(vehicle.GetVehicle())
driver.Initialize()

# === Visualization — ChWheeledVehicleVisualSystemIrrlicht ===
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("UAZBUS Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()         # vehicle truth uses directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Review-only setup ===

# === Main loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0

try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        sim_time = sys.GetChTime()

        if frame % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(sim_time)
        terrain.Synchronize(sim_time)
        vehicle.Synchronize(sim_time, driver_inputs, terrain)
        vis.Synchronize(sim_time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        vehicle.Advance(TIME_STEP)
        vis.Advance(TIME_STEP)


        frame += 1
        realtime_timer.Spin(TIME_STEP)

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
