"""M113 tracked vehicle driving on rigid terrain (PyChrono 9.0.x, Irrlicht).

Models the M113 armored personnel carrier as a tracked vehicle wrapper
(``veh.M113``) rolling forward on a flat rigid terrain patch. The vehicle uses
an NSC (complementarity) contact system — the single-pin track shoe model is
numerically unstable under SMC penalty contact, so non-smooth complementarity
contact with an NSC terrain material is used throughout. Powertrain is a
SHAFTS engine with a BDS (basic differential) driveline so the sprockets
receive real tractive torque and the tracks actually pull the hull forward.

System type : ChSystemNSC (owned by the veh.M113 wrapper).
Main bodies : tracked-vehicle hull/chassis, two sprockets, idlers, road wheels,
              and ~63/64 single-pin track shoes per side; one rigid terrain patch.
Driver      : pre-programmed data driver (brief settle, then steady throttle).
Expected    : the hull accelerates from rest and translates forward in +X,
              staying upright with both tracks intact and resting on the ground.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh

# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 1e-3                 # integration step (s) — single-pin track needs small dt
SIM_END = 8.0                    # simulated duration (s)
RENDER_FPS = 50.0                # review-video frame rate
TERRAIN_LENGTH = 100.0           # rigid patch X extent (m)
TERRAIN_WIDTH = 100.0            # rigid patch Y extent (m)
TERRAIN_FRICTION = 0.9           # terrain Coulomb friction (grip for the tracks)
TERRAIN_RESTITUTION = 0.01       # terrain restitution (nearly inelastic)
INIT_X = 0.0                     # spawn X (m)
INIT_Y = 0.0                     # spawn Y (m)
INIT_Z = 0.64                    # spawn Z so the track shoes rest on the terrain top
THROTTLE = 1.0                   # steady forward throttle after settle

# Derived constants (precomputed once — never recomputed in the loop).
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)           # precomputed once
init_rot = chrono.QUNIT                                        # identity heading


# === Vehicle === tracked M113 wrapper (NSC contact, SHAFTS engine, BDS driveline)
vehicle = veh.M113()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC: single-pin track is unstable under SMC
vehicle.SetTrackShoeType(veh.TrackShoeType_SINGLE_PIN)
vehicle.SetDrivelineType(veh.DrivelineTypeTV_BDS)      # basic differential -> real tractive torque
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.Initialize()

# Visualization detail of the wrapper-created subsystems (after Initialize).
vehicle.SetChassisVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSprocketVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetIdlerVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetRoadWheelVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetTrackShoeVisualizationType(chrono.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.M113 wrapper) ===
sys = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()        # cache: main hull rigid body, reused every step
veh_obj = vehicle.GetVehicle()            # cache: ChTrackedVehicle handle, reused every step
# sprockets/idlers/road-wheels/track-shoes + their revolute joints are created
# inside the wrapper per side; the terrain RigidTerrain patch is added below.

# Required for any scene with contact/collision: set the Bullet collision system
# on the wrapper-owned system AFTER Initialize (the tracks + terrain collide).
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain === flat rigid patch with defined friction / restitution (NSC)
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Track-shoe counts per side — cached once for the Synchronize terrain-force buffers.
n_left = veh_obj.GetNumTrackShoes(veh.LEFT)    # cache: fetched once, reused every step
n_right = veh_obj.GetNumTrackShoes(veh.RIGHT)  # cache: fetched once, reused every step
shoe_forces_left = veh.TerrainForces(n_left)   # precomputed once
shoe_forces_right = veh.TerrainForces(n_right)  # precomputed once

# === Driver === pre-programmed open-loop schedule (settle, then steady throttle)
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.3, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.0, THROTTLE, 0.0, 0.0),
    veh.DataDriverEntry(SIM_END, 0.0, THROTTLE, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Visualization === tracked-vehicle Irrlicht window: chase cam + sky + lights + logo
vis = veh.ChTrackedVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("M113 Tracked Vehicle on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow point, distance, height
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(veh_obj)


# === Main loop === render-cadence outer loop; tracked Synchronize/Advance inner
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = sys.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, shoe_forces_left, shoe_forces_right)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)     # advances the wrapper-owned ChSystem
            vis.Advance(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    # Always release the render device so partial state is flushed on any exit.
    vis.GetDevice().closeDevice()
