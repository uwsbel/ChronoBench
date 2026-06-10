"""MAN 5t military truck driving over rigid hilly terrain (NSC contact).

Models the MAN_5t catalog wheeled vehicle spawned on the left side of a rigid
terrain patch whose surface is generated from a grayscale height map (rolling
hills), textured with grass. The system is NSC; the terrain is a rigid
heightmap body and the vehicle uses TMEASY tires. An interactive Irrlicht driver
(keyboard) controls the truck in real time; the chase camera follows the
chassis. Expected behavior: the truck rests on the undulating hill surface and,
when driven, climbs/descends the terrain rises without sinking through.
"""

import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
time_step = 2e-3                       # integrator step (s)
sim_end = 12.0                         # bounded recording horizon (s)
render_fps = 50.0                      # review frame cadence

INIT_LOC = chrono.ChVector3d(-20.0, 0.0, 1.5)   # prompt: spawn on the hill terrain
INIT_ROT = chrono.ChQuaterniond(1, 0, 0, 0)     # identity heading (+X forward)

TERRAIN_L = 100.0                      # height-map patch length (X, m)
TERRAIN_W = 100.0                      # height-map patch width  (Y, m)
HMIN = 0.0                             # height-map min elevation (m)
HMAX = 4.0                             # height-map max elevation (m) -> rolling hills
TIRE_RADIUS = 0.55                     # MAN_5t tire radius (m), for footprint assert

render_step_size = 1.0 / render_fps                       # precomputed once
render_steps = max(1, round(render_step_size / time_step))  # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === MAN_5t catalog wrapper owns its own NSC ChSystem
vehicle = veh.MAN_5t()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(INIT_LOC, INIT_ROT))
vehicle.SetTireType(veh.TireModelType_TMEASY)          # deformable-capable tire on rigid terrain
vehicle.SetTireStepSize(time_step)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.MAN_5t wrapper) ===
system = vehicle.GetSystem()                       # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()                 # cache: main chassis body, reused for camera/log
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
# spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension/steering inside wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# === Terrain === rigid body whose surface is a grayscale height map (hills)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()          # NSC material matches the NSC system
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,                               # patch centered at world origin
    veh.GetDataFile("terrain/height_maps/test64.bmp"),  # rolling-hill height map
    TERRAIN_L, TERRAIN_W,
    HMIN, HMAX,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # prompt: grass texture
terrain.Initialize()

# === Footprint check === confirm the wheels start on the hill surface, not through it
veh_obj = vehicle.GetVehicle()                     # cache: fetched once for the spindle query
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
ground_z = terrain.GetHeight(INIT_LOC)             # terrain elevation under the spawn point
assert wheel_bottom_z >= ground_z - 0.3, (
    f"vehicle sinks into hill: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs ground z={ground_z:.3f}; raise INIT_LOC.z"
)

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("MAN 5t on rigid hill terrain")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 10.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                          # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())

# === Driver === interactive (keyboard) driver bound to the visual system
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time render-throttled Synchronize/Advance of the stack
realtime_timer = chrono.ChRealtimeStepTimer()
try:
    step_number = 0
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:        # throttled rendering
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
        vehicle.Advance(time_step)                 # advances the wrapper-owned system
        vis.Advance(time_step)

        step_number += 1
        realtime_timer.Spin(time_step)             # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:          # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
