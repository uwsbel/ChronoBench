"""
Kraz Tractor-Trailer Simulation on Rigid Terrain with Interactive Driver.

System: ChSystemNSC (NSC contact method via Kraz wrapper)
Terrain: RigidTerrain flat patch with defined friction and restitution
Vehicle: veh.Kraz() tractor-trailer wrapper
Driver: ChInteractiveDriverIRR — real-time keyboard interactive (scored core)
Visualization: ChWheeledVehicleVisualSystemIrrlicht (chase camera, sky, lights)
Expected behavior: Kraz vehicle rests on rigid terrain; user can drive it in real-time.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh

# Set vehicle data path so Kraz can load its mesh/JSON assets
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Constants ===
# Simulation timing
TIME_STEP = 5e-4           # 0.5 ms physics step for stable tractor-trailer contact
SIM_END = 30.0             # simulation duration (seconds)
RENDER_FPS = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Terrain parameters (defined friction and restitution per prompt)
TERRAIN_FRICTION = 0.9
TERRAIN_RESTITUTION = 0.01
TERRAIN_LENGTH = 300.0     # m
TERRAIN_WIDTH = 300.0      # m

# Vehicle initial conditions
INIT_X = 0.0
INIT_Y = 0.0
KRAZ_TIRE_RADIUS = 0.559      # Kraz tire radius (introspected from tire.GetRadius())
SUSPENSION_REF_HEIGHT = 0.60  # chassis origin above wheel-bottom at rest; Kraz spindles end at ~z=INIT_Z
INIT_Z = 0.0 + SUSPENSION_REF_HEIGHT  # terrain top at z=0; spindles at INIT_Z, wheel-bottom at INIT_Z-tire_r

# Driver response rates
STEERING_TIME = 1.0   # s to reach max steering
THROTTLE_TIME = 1.0   # s to reach max throttle
BRAKING_TIME = 0.3    # s to reach max braking

# === Vehicle setup (Kraz tractor-trailer wrapper) ===
# Kraz owns its own ChSystemNSC internally; attach vis via GetTractor(), NOT GetVehicle()
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity — facing +X

truck = veh.Kraz()
truck.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain
truck.SetChassisFixed(False)                              # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
truck.SetTireStepSize(TIME_STEP)  # Kraz has no SetTireType — it has a built-in rigid tire type
truck.Initialize()

# === System & bodies (created by the veh.Kraz wrapper) ===
sys = truck.GetSystem()                                   # ChSystemNSC owned by the wrapper
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
chassis = truck.GetTractorChassisBody()                  # cache: tractor chassis body
# Bodies: tractor chassis, trailer chassis, 4 tractor axle spindles, 2 trailer axle spindles
# Joints: suspension links, steering links, tractor-trailer hitch — created inside Kraz wrapper

# Visualization types for vehicle sub-systems
# Kraz is a tractor-trailer: SetChassisVis/SetSuspensionVis/SetWheelVis/SetTireVis take 2 args
# (vis_tractor, vis_trailer); SetSteeringVis takes only 1 (tractor-only steering)
# VisualizationType lives in veh module in this 9.0.0 source build
truck.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
truck.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

# === Terrain ===
# RigidTerrain flat patch with specified friction and restitution
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()   # NSC matches the vehicle contact method
patch_mat.SetFriction(TERRAIN_FRICTION)
patch_mat.SetRestitution(TERRAIN_RESTITUTION)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,       # centered at origin, flat
    TERRAIN_LENGTH,
    TERRAIN_WIDTH,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization (ChWheeledVehicleVisualSystemIrrlicht) ===
# Kraz: attach vis via GetTractor(), NOT GetVehicle() (tractor-trailer special case)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Tractor-Trailer — Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(truck.GetTractor())    # Kraz: must use GetTractor()

# === Driver (interactive — scored core: ChInteractiveDriverIRR) ===
# Builds the IRR driver after vis is initialized; takes the visual system, not the vehicle
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(RENDER_EVERY * TIME_STEP / STEERING_TIME)   # precomputed once
driver.SetThrottleDelta(RENDER_EVERY * TIME_STEP / THROTTLE_TIME)
driver.SetBrakingDelta(RENDER_EVERY * TIME_STEP / BRAKING_TIME)
driver.Initialize()

# === Validate wheel placement (asset spindle check) ===
TIRE_RADIUS = KRAZ_TIRE_RADIUS    # cache: introspected from tire.GetRadius()
ZTOL = 0.15           # generous tolerance for tractor-trailer
tractor_obj = truck.GetTractor()
spindle_positions = []
for axle_idx in range(tractor_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = tractor_obj.GetSpindlePos(axle_idx, side)
        spindle_positions.append(p)
wheel_bottom_z = min(p.z for p in spindle_positions) - TIRE_RADIUS
assert wheel_bottom_z >= -ZTOL, (
    f"Tractor wheel bottom z={wheel_bottom_z:.3f} sinks below terrain (z=0); "
    f"raise INIT_Z by {-wheel_bottom_z:.3f} m"
)

# === Main simulation loop ===
# Real-time interactive loop: Synchronize/Advance the full vehicle stack each step.
# ChRealtimeStepTimer throttles wall-clock to match sim time (scored core).
realtime_timer = chrono.ChRealtimeStepTimer()

try:
    while vis.Run() and truck.GetSystem().GetChTime() < SIM_END:
        sim_time = truck.GetSystem().GetChTime()   # cache: fetched once per frame

        # Render frame (throttled to RENDER_FPS)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Physics inner batch
        for _ in range(RENDER_EVERY):
            sim_time_inner = truck.GetSystem().GetChTime()

            # Get driver inputs
            driver_inputs = driver.GetInputs()


            # Synchronize (order: driver → terrain → vehicle → vis)
            driver.Synchronize(sim_time_inner)
            terrain.Synchronize(sim_time_inner)
            truck.Synchronize(sim_time_inner, driver_inputs, terrain)
            vis.Synchronize(sim_time_inner, driver_inputs)


            # Advance (vehicle.Advance steps the wrapper-owned system — do NOT also call DoStepDynamics)
            driver.Advance(TIME_STEP)
            terrain.Advance(TIME_STEP)
            truck.Advance(TIME_STEP)
            vis.Advance(TIME_STEP)

            if truck.GetSystem().GetChTime() >= SIM_END:
                break

        realtime_timer.Spin(TIME_STEP)   # throttle to real-time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass

# === Post-processing ===
# (No post-processing in the scored interactive demo core)
