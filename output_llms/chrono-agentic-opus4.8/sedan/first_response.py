"""BMW E90 Sedan dynamics on rigid terrain (PyChrono 9.0.0, Irrlicht).

Models a BMW E90 catalog sedan driving on a flat rigid-terrain patch. The
vehicle uses an NSC contact method (rigid-terrain default), a TMEASY tire model,
and full mesh visualization. It is controlled by an interactive driver
(ChInteractiveDriverIRR) bound to the vehicle's Irrlicht visual system, enabling
real-time steering / throttle / braking. The scene is rendered with a chase
camera, a directional light, a skybox, a Pychrono logo, and a textured terrain
patch. Expected behavior: the sedan rests on the terrain and responds to driver
inputs, rolling forward and steering under throttle.

System type: NSC (rigid-terrain catalog vehicle).
Main bodies: sedan chassis + 4 wheels/spindles (wrapper-created), rigid terrain.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants and derived spawn pose
time_step = 2e-3                      # integration step (s)
sim_end = 12.0                        # bounded recording horizon (s)
render_step_size = 1.0 / 50.0         # 50 FPS render cadence
terrain_length = 200.0                # rigid terrain patch X size (m)
terrain_width = 100.0                 # rigid terrain patch Y size (m)
init_loc = chrono.ChVector3d(0, 0, 0.5)       # chassis-origin spawn (geometric center)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity orientation, facing +X
TIRE_RADIUS = 0.33                    # approx BMW E90 tire radius (m), for footprint check
ZTOL = 0.10                           # allowed wheel-bottom clearance vs terrain top (m)

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())          # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')      # locate vehicle data files

# === Vehicle === BMW E90 sedan catalog wrapper (owns its ChSystem)
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)          # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)       # no chassis collision box
vehicle.SetChassisFixed(False)                                # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)                 # prompt: TMEASY tire model
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

# Mesh visualization for every subsystem (chassis, suspension, wheels, tires).
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
chassis = vehicle.GetChassisBody()                 # cache: main chassis rigid body, reused below
# wheels/spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension/steering links (wrapper-created)
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass

# Footprint sanity: wheels must rest on (not through) the terrain top at z=0.
veh_obj = vehicle.GetVehicle()                     # cache: vehicle handle reused for spindles
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= 0.0 - ZTOL, (
    f"vehicle sinks into terrain: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs terrain top z=0.000; raise init_loc.z by {0.0 - wheel_bottom_z:.3f} m"
)

# === Terrain === flat rigid patch with texture + color (NSC material)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # customizable terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht: window + chase cam + sky + light + logo
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 Sedan on Rigid Terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)   # chase camera behind chassis
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # customizable logo
vis.AddSkyBox()                                                   # outdoor sky backdrop
vis.AddLightDirectional()                                         # directional lighting (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive real-time driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0   # seconds 0 -> +1 steering
throttle_time = 1.0   # seconds 0 -> +1 throttle
braking_time = 0.3    # seconds 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance over the full subsystem stack
render_steps = math.ceil(render_step_size / time_step)   # precomputed once: steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()            # cache: spin to wall-clock each step


step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:            # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        driver_inputs = driver.GetInputs()


        driver.Synchronize(time)
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(time_step)
        terrain.Advance(time_step)
        vehicle.Advance(time_step)
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)
except (RuntimeError, ValueError) as exc:              # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
