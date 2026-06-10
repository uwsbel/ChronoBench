"""Full HMMWV on flat rigid terrain — interactive real-time driving demo.

Models a complete HMMWV (High Mobility Multipurpose Wheeled Vehicle) created via
the `veh.HMMWV_Full` catalog wrapper and driven over a flat `veh.RigidTerrain`
patch. System type: NSC (rigid-terrain catalog vehicle). Main bodies: the wrapper
chassis + four suspension/wheel assemblies (created internally) plus one rigid
terrain patch. Tire model: TMEASY. Visualization: primitive vehicle components in
an Irrlicht chase-camera window. Control: an interactive (keyboard) driver for
steering, throttle, and braking. Expected behavior: the vehicle rests on the
terrain and drives in real time, rendering at 50 frames per second; under the
review maneuver it accelerates forward and stays on the textured ground.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics / timing constants (no bare literals below)
TIME_STEP = 1e-3                 # integration step (s)
RENDER_FPS = 50.0                # real-time render cadence (frames per second)
TIRE_RADIUS = 0.467              # HMMWV tire radius (m), for the footprint assert
SUSPENSION_REF_HEIGHT = 0.5      # chassis origin above wheel-bottom at rest (m)
TERRAIN_LENGTH = 100.0           # rigid terrain patch X size (m)
TERRAIN_WIDTH = 100.0            # rigid terrain patch Y size (m)
TERRAIN_TOP_Z = 0.0              # flat terrain surface height (m)

INIT_X = 0.0
INIT_Y = 0.0
INIT_Z = TERRAIN_TOP_Z + SUSPENSION_REF_HEIGHT   # derived chassis spawn height
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, INIT_Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # identity: facing +X


# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV via the catalog wrapper (owns its own ChSystemNSC)
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)          # prompt: TMEASY tire model
hmmwv.SetTireStepSize(TIME_STEP)
hmmwv.Initialize()

# Primitive visualization for all vehicle components (prompt: primitive viz)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                           # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = hmmwv.GetChassisBody()    # cache: main chassis rigid body, reused for camera target
# wheels/spindles: hmmwv.GetVehicle().GetAxle(i) ...; joints: suspension + steering links (wrapper)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())   # report total vehicle mass

# === Footprint check === assert wheels rest ON the terrain (not through it)
veh_obj = hmmwv.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= TERRAIN_TOP_Z - 0.05, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={TERRAIN_TOP_Z:.3f}; raise SUSPENSION_REF_HEIGHT"
)

# === Terrain === flat rigid patch with friction + a tiled texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht chase-camera window
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()            # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive keyboard driver (steering / throttle / braking)
render_step_size = 1.0 / RENDER_FPS
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)   # 1 s to reach full steering
driver.SetThrottleDelta(render_step_size / 1.0)   # 1 s to reach full throttle
driver.SetBrakingDelta(render_step_size / 0.3)    # 0.3 s to reach full braking
driver.Initialize()

# === Main loop === real-time render at 50 fps; full subsystem Synchronize/Advance
render_steps = math.ceil(render_step_size / TIME_STEP)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
try:
    while vis.Run():
        time = system.GetChTime()

        if step_number % render_steps == 0:          # throttled rendering at 50 fps
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(TIME_STEP)
        terrain.Advance(TIME_STEP)
        hmmwv.Advance(TIME_STEP)          # advances the wrapper-owned system
        vis.Advance(TIME_STEP)

        step_number += 1
        realtime_timer.Spin(TIME_STEP)    # spin so wall-clock matches sim time

except (RuntimeError, ValueError) as exc:   # solver divergence / bad vehicle state
    import traceback
    traceback.print_exc()
    raise
