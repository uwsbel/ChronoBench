"""Kraz tractor-semitrailer double-lane-change on flat rigid terrain (PyChrono 9.0.1, Irrlicht).

Models the catalog `veh.Kraz` articulated heavy truck (tractor + semitrailer) driving
on a flat `veh.RigidTerrain` patch. The vehicle wrapper owns a `ChSystemNSC`; collision
is resolved with the Bullet collision system. A scripted `veh.ChDriver` subclass issues
an open-loop ISO double-lane-change steering maneuver (accelerate straight, swerve left
into the adjacent lane, hold, swerve back to the original lane) as a function of
simulation time. The truck spawns at world X = -15 m facing +X and is followed by a
chase camera. Expected behavior: the rig accelerates forward, performs the two lane
changes, and stays upright on the terrain throughout.
"""

import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Constants === geometry / timing / maneuver parameters (no bare literals downstream)
TIME_STEP = 2e-3                         # integration step (s)
TIRE_STEP = 1e-3                         # tire model substep (s)
SIM_END = 16.0                           # total simulated time (s)
RENDER_FPS = 50.0                        # review-video frame rate

# Initial pose: spawn behind the origin, facing +X (heading along the lane axis).
INIT_X = -15.0
INIT_Y = 0.0
KRAZ_REF_HEIGHT = 0.5588                 # tractor chassis-origin rest height above flat ground
INIT_CLEARANCE = 0.0                     # extra drop clearance above the rest height
INIT_Z = KRAZ_REF_HEIGHT + INIT_CLEARANCE
INIT_LOC = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
INIT_ROT = chrono.QuatFromAngleZ(0.0)    # heading +X

# Flat rigid terrain — large patch so the rig stays on-terrain through both lane changes.
TERRAIN_LENGTH = 300.0
TERRAIN_WIDTH = 60.0
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01

# Chase camera: follow a point ahead of the tractor chassis from well behind and above.
CHASE_TRACK_POINT = chrono.ChVector3d(3.0, 0.0, 2.1)
CHASE_DISTANCE = 25.0
CHASE_HEIGHT = 10.5

# Double-lane-change maneuver schedule (open-loop, time-based).
LANE_THROTTLE = 0.6                      # steady forward throttle once rolling
STEER_AMP = 0.20                         # peak steering input (-1..+1)
T_LAUNCH = 1.0                           # straight-line acceleration phase end (s)
T_SWERVE1 = 4.0                          # begin first swerve (to left lane)
T_HOLD = 7.0                             # settle in left lane
T_SWERVE2 = 9.0                          # begin return swerve (back to right lane)
T_SETTLE = 12.0                          # settled back in original lane


# === Driver === scripted ChDriver subclass issuing the open-loop double lane change
class DoubleLaneChangeDriver(veh.ChDriver):
    """Open-loop driver: ramp throttle, then two opposite half-sine steering pulses."""

    def __init__(self, vehicle):
        super().__init__(vehicle)

    def Synchronize(self, time):
        # Throttle: brief launch ramp, then steady cruise.
        if time < T_LAUNCH:
            self.SetThrottle(LANE_THROTTLE * (time / T_LAUNCH))
        else:
            self.SetThrottle(LANE_THROTTLE)
        self.SetBraking(0.0)

        # Steering: first half-sine pulse left, then an opposite pulse back.
        steer = 0.0
        if T_SWERVE1 <= time < T_HOLD:
            phase = (time - T_SWERVE1) / (T_HOLD - T_SWERVE1)
            steer = STEER_AMP * math.sin(math.pi * phase)
        elif T_SWERVE2 <= time < T_SETTLE:
            phase = (time - T_SWERVE2) / (T_SETTLE - T_SWERVE2)
            steer = -STEER_AMP * math.sin(math.pi * phase)
        self.SetSteering(steer)


# === System & vehicle === Kraz wrapper owns the ChSystemNSC; Bullet collision required
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireStepSize(TIRE_STEP)
vehicle.Initialize()

# Make wrapper-created essentials visible: system, tractor, chassis body, trailer.
system = vehicle.GetSystem()                      # ChSystemNSC owned by the Kraz wrapper
tractor = vehicle.GetTractor()                    # tractor ChWheeledVehicle subsystem
tractor_chassis = vehicle.GetTractorChassisBody() # cache: tractor chassis rigid body, reused below
trailer = vehicle.GetTrailer()                    # semitrailer subsystem (suspension + joints)

# Bullet collision system MUST be set on the vehicle-owned system for terrain/tire contact.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization types — tractor + trailer take TWO args; steering takes ONE.
vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES, chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH, chrono.VisualizationType_MESH)

# === Terrain === flat rigid patch under the rig
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Driver === scripted double-lane-change controller on the tractor
driver = DoubleLaneChangeDriver(vehicle.GetTractor())
driver.Initialize()

# === Visualization === vehicle-aware Irrlicht window + chase camera + sky/lights/logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Double Lane Change")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(CHASE_TRACK_POINT, CHASE_DISTANCE, CHASE_HEIGHT)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(vehicle.GetTractor())
vis.AttachDriver(driver)

# === Main loop === render-cadence outer loop; full Synchronize/Advance subsystem stack
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once



frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)        # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
