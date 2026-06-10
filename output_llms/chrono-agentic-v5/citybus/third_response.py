"""CityBus on flat rigid terrain (NSC) with an interactive driver.

Models a Chrono catalog CityBus wheeled vehicle driving on a flat rigid-terrain
patch. The vehicle uses the Pacejka '89 (PAC89) tire model and a small 5e-4 s
simulation/tire step for solver stability. The terrain patch is textured with the
dirt-road texture. The chassis, wheels, and tires are rendered with mesh assets in
an Irrlicht chase-camera window; the scored core instantiates the interactive
keyboard driver that the real-time catalog-vehicle truths use. Expected behavior:
the bus rests on its wheels on the flat dirt patch and is drivable in real time.
"""

import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / physics constants (no bare literals downstream)
step_size = 5e-4              # simulation step (s) — reduced from 1e-3 for stability
tire_step_size = 5e-4        # tire step (s) — matched to the reduced sim step
sim_end = 12.0               # bounded run length (s)
render_fps = 50.0            # review render cadence
terrain_length = 200.0       # rigid patch X size (m)
terrain_width = 200.0        # rigid patch Y size (m)
init_loc = chrono.ChVector3d(0, 0, 0.5)        # chassis-origin spawn (m)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # identity heading
TIRE_RADIUS = 0.5            # CityBus tire radius (m), for the footprint assert
PATCH_TOP_Z = 0.0            # flat rigid patch top plane (m)

render_step_size = 1.0 / render_fps                       # precomputed once
render_steps = math.ceil(render_step_size / step_size)    # precomputed once

# === Data paths === locate bundled Chrono + vehicle assets (truth components)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# === Vehicle === catalog CityBus, NSC rigid terrain, PAC89 tires
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)          # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)                                # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_PAC89)                  # prompt: Pacejka '89 tire model
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)
bus.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
chassis = bus.GetChassisBody()                            # cache: main chassis rigid body, reused below
# spindles: bus.GetVehicle().GetSpindlePos(axle, side) ; terrain patch body created below
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())       # report total vehicle mass

# Footprint sanity: wheels must rest on (not through) the flat patch after Initialize.
veh_obj = bus.GetVehicle()
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= PATCH_TOP_Z - 0.15, (
    f"vehicle sinks into patch: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs patch top z={PATCH_TOP_Z:.3f}; raise init_loc.z"
)

# === Terrain === flat rigid patch textured as a dirt road
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)  # prompt: dirt road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
terrain.Initialize()

# === Visualization === vehicle-aware Irrlicht window: chase camera + sky + light
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on dirt road (PAC89 tires)")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                 # vehicle truths use a directional light
vis.AttachVehicle(bus.GetVehicle())

# === Driver === interactive keyboard driver bound to the visual system (truth shape)
driver = veh.ChInteractiveDriverIRR(vis)
driver.SetSteeringDelta(render_step_size / 1.0)
driver.SetThrottleDelta(render_step_size / 1.0)
driver.SetBrakingDelta(render_step_size / 0.3)
driver.Initialize()

# === Main loop === real-time Synchronize/Advance stack for the full subsystem set

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:               # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        driver_inputs = driver.GetInputs()

        driver.Synchronize(time)
        terrain.Synchronize(time)
        bus.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        bus.Advance(step_size)
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)
except (RuntimeError, ValueError) as exc:                 # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise

# === Post-processing === assemble review video + plot, then drop frame dirs
