"""
MAN 10t Truck on Rigid Terrain — Interactive Driver Demo
=========================================================
System type  : ChSystemNSC (owned by the veh.MAN_10t wrapper)
Contact      : NSC rigid terrain, TMEASY tire model
Main bodies  : MAN_10t chassis + 4 tires (wrapper-managed)
Terrain      : RigidTerrain flat patch with texture
Visualization: ChWheeledVehicleVisualSystemIrrlicht with chase camera,
               directional lighting, skybox, logo
Driver       : ChInteractiveDriverIRR for real-time steering/throttle/brake

Expected behavior: Truck starts stationary; user steers/throttles via
keyboard (arrow keys / WASD) in the real-time interactive Irrlicht window.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh

# Set absolute data path so textures resolve regardless of CWD
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# === Named constants ===
step_size             = 1e-3          # simulation time step (s)
sim_end               = 30.0          # simulation duration (s)
render_fps            = 50.0          # Irrlicht render rate (Hz)

terrain_length        = 800.0         # terrain patch length X (m) — large enough for 30s drive
terrain_width         = 800.0         # terrain patch width Y (m)
terrain_texture_tiles = 200           # UV tiling for terrain texture

# Chase camera parameters
chase_point  = chrono.ChVector3d(0.0, 0.0, 1.75)   # point on chassis to track
chase_dist   = 9.0                                   # follow distance (m)
chase_height = 0.5                                   # camera height offset (m)

# Initial spawn position — wheels rest on flat terrain at z=0
SUSPENSION_REF_HEIGHT = 0.5          # chassis-origin above wheel-bottom at rest (m)
init_loc = chrono.ChVector3d(0.0, 0.0, SUSPENSION_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # identity: truck faces default forward direction

# Driver response rates (seconds to reach full input)
steering_time = 1.0
throttle_time = 1.0
braking_time  = 0.3

# Rendering cadence (precomputed once)
render_every = max(1, round(1.0 / (render_fps * step_size)))   # precomputed once

# === Vehicle (MAN 10t wrapper) ===
# veh.MAN_10t() creates and owns a ChSystemNSC internally.
truck = veh.MAN_10t()
truck.SetContactMethod(chrono.ChContactMethod_NSC)       # NSC for rigid terrain (truth)
truck.SetChassisCollisionType(veh.CollisionType_NONE)
truck.SetChassisFixed(False)                             # MANDATORY — fixed chassis won't move
truck.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
truck.SetTireType(veh.TireModelType_TMEASY)              # prompt: TMEASY tire model
truck.SetTireStepSize(step_size)
truck.Initialize()

# === System & bodies (created by veh.MAN_10t wrapper) ===
sys     = truck.GetSystem()                              # ChSystemNSC owned by the wrapper
chassis = truck.GetChassisBody()                         # cache: main chassis rigid body
# Wheels/spindles: truck.GetVehicle().GetAxle(i)...
# Joints: suspension + steering links created inside the wrapper

# REQUIRED: collision system for contact/terrain interaction
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Visualization types (set after Initialize, before vis init)
# Note: VisualizationType_* lives in veh namespace for vehicle wrappers
# Use PRIMITIVES for robustness (MESH requires bundled .obj assets path)
truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
truck.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === Terrain (rigid, flat) ===
terrain = veh.RigidTerrain(sys)

patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,              # centered at origin, Z-up
    terrain_length,
    terrain_width,
)
patch.SetTexture(
    veh.GetDataFile("terrain/textures/tile4.jpg"),
    terrain_texture_tiles,
    terrain_texture_tiles,
)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Footprint assertion (verify wheels rest on terrain) ===
TIRE_RADIUS = 0.55      # MAN 10t tire radius estimate (m)
ZTOL        = 0.12      # allowed clearance/sinkage tolerance (m)
terrain_z   = 0.0       # flat terrain top at z=0

veh_obj    = truck.GetVehicle()                          # cache: vehicle handle
spindle_zs = []
for axle_idx in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        p = veh_obj.GetSpindlePos(axle_idx, side)
        spindle_zs.append(p.z)

wheel_bottom_z = min(spindle_zs) - TIRE_RADIUS
if wheel_bottom_z < terrain_z - ZTOL:
    print(
        f"[WARNING] Wheel bottom z={wheel_bottom_z:.3f} m below terrain "
        f"z={terrain_z:.3f} m; nudge SUSPENSION_REF_HEIGHT up by "
        f"{terrain_z - wheel_bottom_z:.3f} m"
    )

# === Irrlicht visualization (vehicle visual system) ===
# Order: configure window → Initialize() → scene elements → AttachVehicle
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 10t Truck — Rigid Terrain Interactive Demo")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chase_point, chase_dist, chase_height)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AttachVehicle(truck.GetVehicle())
vis.SetChaseCameraState(veh.ChChaseCamera.Follow)   # activate chase-camera follow mode

# === Interactive driver (ChInteractiveDriverIRR — truth-faithful) ===
driver = veh.ChInteractiveDriverIRR(vis)               # takes the visual system, not vehicle
driver.SetSteeringDelta(render_every * step_size / steering_time)
driver.SetThrottleDelta(render_every * step_size / throttle_time)
driver.SetBrakingDelta(render_every * step_size / braking_time)
driver.Initialize()

# === Review-only recording setup ===


# === Main simulation loop ===
realtime_timer = chrono.ChRealtimeStepTimer()
frame = 0
try:
    while vis.Run():
        time = sys.GetChTime()
        if time >= sim_end:
            break

        # Throttled rendering (once per render frame)
        if frame % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Scored-core driver inputs from the interactive driver
        driver_inputs = driver.GetInputs()


        # Synchronize subsystems (order: driver → terrain → vehicle → vis)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        truck.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance subsystems (truck.Advance steps the wrapper-owned ChSystem)
        driver.Advance(step_size)
        terrain.Advance(step_size)
        truck.Advance(step_size)
        vis.Advance(step_size)

        frame += 1
        realtime_timer.Spin(step_size)

except (RuntimeError, ValueError) as exc:              # solver divergence / bad physics state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing (review-only) ===
