"""FED-Alpha (FEDA) wheeled vehicle on flat rigid terrain, driven interactively.

System type: NSC (rigid-terrain catalog vehicle, Bullet collision).
Main bodies: the FEDA chassis + four wheels/tires (created by the veh.FEDA
wrapper), and a single flat RigidTerrain patch carrying a custom road texture.
Visualization: Irrlicht chase-camera window with mesh visualization for every
vehicle part. Control: an interactive keyboard driver (steering / throttle /
braking) bound to the visual system.

Expected behavior: the vehicle rests on the textured terrain at spawn and is
steered/accelerated/braked in real time; the render loop runs at 50 FPS while
the vehicle dynamics advance at the fixed physics step.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived spawn pose
step_size = 1e-3                       # fixed physics step (s)
render_fps = 50.0                      # render/update rate requested by the prompt
render_step_size = 1.0 / render_fps    # wall-clock seconds between rendered frames
sim_end = 12.0                         # bounded run length for the recording pass

terrain_length = 200.0                 # rigid patch X size (m)
terrain_width = 200.0                  # rigid patch Y size (m)
terrain_top_z = 0.0                    # terrain surface height (m)

CHASSIS_REF_HEIGHT = 0.5               # FEDA chassis-origin height above ground at rest
init_loc = chrono.ChVector3d(0, 0, terrain_top_z + CHASSIS_REF_HEIGHT)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity: facing +X


# === Data paths === anchor the bundled Chrono + vehicle asset trees
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle === FED-Alpha catalog wrapper (owns its own ChSystem)
feda = veh.FEDA()
feda.SetContactMethod(chrono.ChContactMethod_NSC)    # NSC for rigid terrain (pair with ChContactMaterialNSC)
feda.SetChassisCollisionType(veh.CollisionType_NONE)
feda.SetChassisFixed(False)                          # MANDATORY — fixed chassis never moves
feda.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
feda.SetTireType(veh.TireModelType_PAC02)            # prompt: tire model on rigid road
feda.SetTireStepSize(step_size)
feda.Initialize()

# Mesh visualization for ALL vehicle parts (prompt: mesh visualization type)
feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)
feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
feda.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.FEDA wrapper) ===
system = feda.GetSystem()                            # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
chassis = feda.GetChassisBody()                      # cache: main chassis rigid body, reused below
print("VEHICLE MASS: ", feda.GetVehicle().GetMass())  # report total vehicle mass (truth diagnostic)

# Footprint check: wheel bottoms must rest on (not through) the terrain.
veh_obj = feda.GetVehicle()                          # cache: vehicle handle, reused for spindle query
TIRE_RADIUS = 0.499                                  # FEDA tire radius (m), from wheel geometry
ZTOL = 0.10                                          # allowed wheel-bottom clearance vs terrain top
spindle_z = [veh_obj.GetSpindlePos(a, s).z
             for a in range(veh_obj.GetNumberAxles()) for s in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(spindle_z) - TIRE_RADIUS
assert wheel_bottom_z >= terrain_top_z - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z={terrain_top_z:.3f}; raise CHASSIS_REF_HEIGHT")

# === Terrain === single flat rigid patch with a custom road texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase cam + sky + light + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("FED-Alpha on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 8.0, 0.5)   # follow chassis from behind/above
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                             # vehicle truths use a directional light
vis.AddGrid(1.0, 1.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))            # ground reference grid
vis.AttachVehicle(feda.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)      # seconds 0 -> +1 steering
driver.SetThrottleDelta(render_step_size / 1.0)      # seconds 0 -> +1 throttle
driver.SetBrakingDelta(render_step_size / 0.3)       # seconds 0 -> +1 braking
driver.Initialize()

# === Main loop === throttled 50 FPS rendering + full subsystem Synchronize/Advance
render_steps = math.ceil(render_step_size / step_size)   # precomputed once: physics steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()


frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:             # throttled rendering at 50 FPS
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        feda.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        feda.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)                  # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:               # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
