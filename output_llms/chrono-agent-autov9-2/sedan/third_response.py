"""Sedan driving on a highway mesh with closed-loop speed control.

Model: a catalog ``veh.Sedan`` wheeled vehicle (NSC system owned by the wrapper)
spawned on a rigid highway-mesh terrain. The road mesh runs along the world Y
axis (Y in [-75, +75], narrow in X), so the car is spawned near the south end
facing +Y, down the length of the road, to keep it clear of the side barriers.

Control: a PID throttle controller tracks a constant reference forward speed
from the measured chassis speed (throttle from the speed error, braking when the
car is above the reference), and the steering is ramped smoothly from 0 to a
small hold value over a 5 second response time so the car eases into its line
rather than snapping the wheel. The vehicle subsystem stack is advanced with the
Synchronize/Advance contract (the wrapper's Advance steps the owned ChSystem).

Expected behavior: the sedan accelerates from rest, the PID settles the forward
speed near the reference, and the car tracks straight down the highway between
the barriers.
"""

# === Imports ===
import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Named constants (geometry / physics / control) ===
# Time stepping — decreased step + render step for finer control.
TIME_STEP = 1.0e-3                       # physics step (s) — fine integration
RENDER_FPS = 50.0                        # review render cadence (frames/s)
SIM_END = 12.0                           # total simulated time (s)

# Highway mesh terrain assets (collision + visual share the road geometry).
HIGHWAY_COL = "synchrono/meshes/Highway_col.obj"   # collision mesh
HIGHWAY_VIS = "synchrono/meshes/Highway_vis.obj"   # visual mesh

# Vehicle spawn: south end of the road, facing +Y down the road length.
ROAD_HALF_LEN_Y = 75.0                   # mesh Y half-extent (road runs along Y)
VEH_INIT_X = 5.5                         # center of a right-side lane (median X=0, edge X~11.6)
VEH_INIT_Y = -ROAD_HALF_LEN_Y + 12.0     # start near south end, room to drive
VEH_INIT_Z = 0.5                         # chassis-origin height so wheels rest on road
HEADING_YAW = math.pi / 2.0              # +90 deg about Z -> chassis +X faces world +Y
TIRE_RADIUS = 0.336                      # Sedan tire radius (m), from wheel geometry
ROAD_TOP_Z = 0.0                         # drivable road surface height near spawn
ZTOL = 0.20                              # allowed wheel-bottom clearance vs road top

# Speed controller: reference speed + PID gains for throttle.
REF_SPEED = 8.0                          # reference forward speed (m/s)
KP, KI, KD = 0.40, 0.05, 0.0             # PID gains on the speed error
STEER_RAMP_TIME = 5.0                    # increased steering response time (s)
STEER_HOLD = -0.01                       # gentle steady steering away from the median (in-lane)

# === Derived constants (precomputed once) ===
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
init_loc = chrono.ChVector3d(VEH_INIT_X, VEH_INIT_Y, VEH_INIT_Z)
init_rot = chrono.QuatFromAngleZ(HEADING_YAW)
inv_ramp = 1.0 / STEER_RAMP_TIME                               # precomputed once



# === Vehicle (Sedan wrapper owns its ChSystem + chassis/spindles/joints) ===
# The wrapper creates the system, chassis rigid body, four spindles, the
# suspension + steering joints, the powertrain, and the tires internally.
vehicle = veh.Sedan()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireType(veh.TireModelType_TMEASY)   # slip/grip tire for rigid road
vehicle.SetTireStepSize(TIME_STEP)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(chrono.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(chrono.VisualizationType_MESH)
vehicle.SetTireVisualizationType(chrono.VisualizationType_MESH)

# === System & bodies (created by the veh.Sedan wrapper) ===
system = vehicle.GetSystem()                 # ChSystemNSC owned by the wrapper
chassis = vehicle.GetChassisBody()           # cache: main chassis body, reused every step
veh_obj = vehicle.GetVehicle()               # cache: vehicle subsystem handle, reused every step
# spindles: veh_obj.GetSpindlePos(axle, side); joints: suspension + steering links internal

# Collision system for vehicle/terrain contact (Bullet narrow-phase).
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Terrain (rigid highway mesh) ===
# Road geometry loaded as a single mesh patch; tires contact the collision mesh.
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    chrono.GetChronoDataFile(HIGHWAY_COL),   # collision mesh file
    True,                                    # connected mesh
    0.0,                                     # sweep sphere radius
    False,                                   # collision mesh not drawn here
)
terrain.Initialize()

# Visual road mesh as a separate fixed body so the drive surface is visible.
vis_trimesh = chrono.ChTriangleMeshConnected()
vis_trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile(HIGHWAY_VIS), True, True)
road_vis_shape = chrono.ChVisualShapeTriangleMesh()
road_vis_shape.SetMesh(vis_trimesh)
road_vis_shape.SetMutable(False)
road_vis_body = chrono.ChBody()
road_vis_body.SetFixed(True)
road_vis_body.SetPos(chrono.ChVector3d(0, 0, 0))
road_vis_body.AddVisualShape(road_vis_shape, chrono.ChFramed())
system.AddBody(road_vis_body)

# Assert the wheels start resting on (not far through) the road surface.
spindle_world = []
for axle in range(veh_obj.GetNumberAxles()):
    for side in (veh.LEFT, veh.RIGHT):
        spindle_world.append(veh_obj.GetSpindlePos(axle, side))
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= ROAD_TOP_Z - ZTOL, (
    f"vehicle sinks into road: wheel bottom z={wheel_bottom_z:.3f} "
    f"vs road top z={ROAD_TOP_Z:.3f}; raise VEH_INIT_Z by "
    f"{ROAD_TOP_Z - wheel_bottom_z:.3f} m"
)

# === Visualization === full Irrlicht scene: window + chase cam + sky + lights + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Sedan on Highway — PID speed control")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.6), 8.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddGrid(2.0, 2.0, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid
vis.AttachVehicle(veh_obj)

# === Driver inputs (scripted: PID throttle + ramped steering) ===
driver_inputs = veh.DriverInputs()
driver_inputs.m_steering = 0.0
driver_inputs.m_throttle = 0.0
driver_inputs.m_braking = 0.0

# PID integral / derivative state for the speed loop.
speed_err_int = 0.0
prev_speed_err = 0.0

# === Main loop === throttled render outer loop; PID + ramp inner physics batch
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            sim_time = system.GetChTime()
            speed = veh_obj.GetSpeed()   # measured forward speed (m/s)

            # PID throttle on the speed error.
            err = REF_SPEED - speed
            speed_err_int += err * TIME_STEP
            d_err = (err - prev_speed_err) / TIME_STEP
            prev_speed_err = err
            cmd = KP * err + KI * speed_err_int + KD * d_err
            driver_inputs.m_throttle = max(0.0, min(1.0, cmd))
            driver_inputs.m_braking = max(0.0, min(1.0, -cmd))

            # Steering ramp: 0 -> hold over STEER_RAMP_TIME seconds.
            ramp = min(1.0, sim_time * inv_ramp)
            driver_inputs.m_steering = STEER_HOLD * ramp


            terrain.Synchronize(sim_time)
            vehicle.Synchronize(sim_time, driver_inputs, terrain)
            vis.Synchronize(sim_time, driver_inputs)

            terrain.Advance(TIME_STEP)
            vehicle.Advance(TIME_STEP)   # advances the wrapper-owned system
            vis.Advance(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
