"""HMMWV wheeled vehicle on flat rigid terrain (PyChrono / Irrlicht).

Models the full HMMWV (`veh.HMMWV_Full`) driving on a flat rigid-terrain patch.
System type: NSC (rigid-terrain catalog vehicle). Main bodies: the HMMWV chassis,
four spindles/wheels with FIALA tires, and a rigid ground patch. The vehicle is
spawned at (1, 0, 0.5); vehicle parts are drawn with PRIMITIVE visualization, the
chassis carries a MESH collision shape, and the wheels run the FIALA tire model.
An interactive (keyboard) driver controls steering/throttle/braking; the vehicle
is expected to rest on the terrain and respond to driver inputs without sinking.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2e-3                 # integration step (s)
sim_end = 12.0                   # bounded recording duration (s)
render_fps = 50.0                # review render cadence (frames/s)

terrain_length = 200.0           # rigid patch X extent (m)
terrain_width = 100.0            # rigid patch Y extent (m)

init_loc = chrono.ChVector3d(1, 0, 0.5)          # prompt: spawn at (1, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # identity orientation
TIRE_RADIUS = 0.464              # HMMWV tire radius (m), for footprint assert
ZTOL = 0.1                       # allowed wheel-bottom clearance vs terrain top

vis_type = veh.VisualizationType_PRIMITIVES      # prompt: PRIMITIVES for parts
chassis_coll_type = veh.CollisionType_MESH       # prompt: chassis collision MESH
tire_model = veh.TireModelType_FIALA             # prompt: FIALA tire model

# === Data paths === anchor bundled Chrono + vehicle asset trees (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === full HMMWV wrapper (owns its ChSystemNSC); configure then init
hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(chassis_coll_type)     # MESH chassis collision
hmmwv.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
hmmwv.SetTireType(tire_model)                        # FIALA tire force model
hmmwv.SetTireStepSize(time_step)
hmmwv.Initialize()

# Visualization types for the vehicle subsystems (after Initialize)
hmmwv.SetChassisVisualizationType(vis_type)
hmmwv.SetSuspensionVisualizationType(vis_type)
hmmwv.SetSteeringVisualizationType(vis_type)
hmmwv.SetWheelVisualizationType(vis_type)
hmmwv.SetTireVisualizationType(vis_type)

# === System & bodies (created by the veh.HMMWV_Full wrapper) ===
system = hmmwv.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = hmmwv.GetChassisBody()           # cache: main chassis rigid body, reused
# wheels/spindles: hmmwv.GetVehicle().GetSpindlePos(axle, side); joints: suspension
# + steering links created inside the wrapper; terrain patch body added below.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED (contact)
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total mass

# === Footprint assert === verify wheels rest on (not through) the terrain top
veh_obj = hmmwv.GetVehicle()               # cache: vehicle handle, reused below
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= 0.0 - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z=0.0; raise init Z by {0.0 - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch centered at origin, NSC contact material
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                  # vehicle truths use a directional light
vis.AttachVehicle(hmmwv.GetVehicle())

# === Driver === interactive (keyboard) driver bound to the visual system
render_step_size = 1.0 / render_fps                    # precomputed once
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time subsystem Synchronize/Advance stack
render_steps = math.ceil(render_step_size / time_step)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        hmmwv.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:        # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
