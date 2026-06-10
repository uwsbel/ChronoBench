"""CityBus driven by a pre-programmed data driver on flat rigid terrain.

Model
-----
- System: ChSystemNSC owned by the veh.CityBus wrapper (SMC contact method),
  rigid wheeled-vehicle dynamics with TMEASY tires on a flat RigidTerrain patch.
- Main bodies: bus chassis + four wheel spindles (created inside the wrapper),
  one large flat rigid terrain patch acting as the road/ground.
- Control: an open-loop veh.ChDataDriver replays a fixed throttle/steering/brake
  schedule — full throttle from 0.1 s, then a hard left steer (0.7) held from
  0.5 s onward. No human-in-the-loop input.

Expected behavior
------------------
The bus launches forward under full throttle, then commits to a sustained hard
turn once the steering ramps to 0.7, carving a curved trajectory across the
patch. The terrain patch is intentionally large so the turning arc stays on the
road for the whole run.
"""

import math
import os
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


# === Constants === geometry / timing / control schedule (no bare literals below)
time_step = 1e-3                       # integration step (s)
tire_step_size = 1e-3                  # TMEASY tire substep (s)
sim_end = 8.0                          # total simulated time (s)
render_fps = 50.0                      # review-render cadence (frames/s)

INIT_X = 0.0                           # spawn X on the patch (m)
INIT_Y = 0.0                           # spawn Y on the patch (m)
CHASSIS_REF_HEIGHT = 0.7               # bus chassis-origin height above road at rest (m)
init_z = CHASSIS_REF_HEIGHT            # road top sits at z = 0
init_loc = chrono.ChVector3d(INIT_X, INIT_Y, init_z)
init_rot = chrono.QUNIT

# The bus accelerates then turns hard, so it sweeps a wide arc -> large patch.
TERRAIN_LENGTH = 200.0                 # patch X extent (m)
TERRAIN_WIDTH = 200.0                  # patch Y extent (m)

ROAD_TOP_Z = 0.0                       # flat road surface height (m)
TIRE_RADIUS = 0.5                      # CityBus tire radius (m), for footprint check
ZTOL = 0.10                            # allowed wheel-bottom clearance/overlap (m)

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === Vehicle (CityBus wrapper owns its ChSystem) ===
# The wrapper internally creates the ChSystemNSC, the chassis rigid body, the
# four wheel/spindle bodies, and all suspension + steering joints.
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_SMC)
bus.SetChassisCollisionType(veh.CollisionType_NONE)
bus.SetChassisFixed(False)
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire so it actually drives + turns
bus.SetTireStepSize(tire_step_size)
bus.Initialize()

bus.SetChassisVisualizationType(chrono.VisualizationType_MESH)
bus.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(chrono.VisualizationType_MESH)
bus.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.CityBus wrapper) ===
system = bus.GetSystem()                 # ChSystemNSC owned by the wrapper
# Vehicle scene has wheel/terrain contact -> Bullet narrowphase is REQUIRED.
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chassis = bus.GetChassisBody()           # cache: main chassis body, reused every step
veh_obj = bus.GetVehicle()               # cache: ChWheeledVehicle handle, reused below
# wheels/spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering
# links live inside the wrapper; terrain patch body is created just below.

# === Terrain === one large flat rigid patch as the road surface
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch_mat.SetYoungModulus(2e7)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, TERRAIN_LENGTH, TERRAIN_WIDTH)
patch.SetTexture(veh.GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Footprint check: wheels must rest ON the road, not sink through it.
spindle_world = [veh_obj.GetSpindlePos(axle, side)
                 for axle in range(veh_obj.GetNumberAxles())
                 for side in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
    f"bus sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs road top z={ROAD_TOP_Z:.3f}; raise CHASSIS_REF_HEIGHT by "
    f"{ROAD_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Driver === open-loop data driver replaying a fixed input schedule
# DataDriverEntry(time, steering, throttle, braking, gear): full throttle from
# 0.1 s, then a hard left steer (0.7) committed from 0.5 s onward.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0, 0.0),
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0, 0.0),
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0, 0.0),
])
driver = veh.ChDataDriver(veh_obj, driver_data)
driver.Initialize()

# === Visualization === full vehicle-aware Irrlicht scene: window + sky + chase cam + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus - data driver hard turn")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 14.0, 0.6)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-15.0, -15.0, 8.0), chrono.ChVector3d(0.0, 0.0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 50, 50,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.01), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid on the road
vis.AttachVehicle(veh_obj)
vis.AttachDriver(driver)

# === Main loop === throttled render + Synchronize/Advance subsystem stepping


try:

    frame = 0
    while vis.Run() and system.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()
            driver_inputs = driver.GetInputs()
            driver.Synchronize(sim_time)
            terrain.Synchronize(sim_time)
            bus.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)
            driver.Advance(time_step)
            terrain.Advance(time_step)
            bus.Advance(time_step)        # advances the wrapper-owned system
            vis.Advance(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    # always report how far the run advanced, even if a step diverged
    print(f"CityBus run reached t={system.GetChTime():.3f} s of {sim_end:.3f} s")

# === Post-processing === assemble review video + plot, then drop raw frames
