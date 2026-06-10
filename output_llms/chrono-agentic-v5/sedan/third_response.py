"""BMW E90 sedan cruising on a highway mesh with closed-loop speed control.

Model: a single catalog BMW_E90 wheeled sedan (NSC contact, rigid terrain)
driven across a triangle-mesh highway patch. The chassis spawn pose is set
explicitly. Longitudinal control is a PID throttle controller that tracks a
constant reference speed from the measured chassis speed error; lateral control
ramps the steering command to its target over a 5 second response time, giving a
gentle, finely-controlled lane change. Small simulation and render step sizes are
used for finer control resolution.

Expected behavior: the sedan accelerates from rest, the PID throttle drives the
chassis speed toward the reference speed, and the steering eases in over 5 s so
the car curves smoothly while staying on the highway surface.
"""

import os
import math

import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# === Parameters === geometry / control / timing constants (no bare literals downstream)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())        # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')    # locate vehicle data files

step_size = 1e-3            # decreased simulation step size for finer control
render_step_size = 1.0 / 100.0   # decreased render step size for finer control
sim_end = 12.0

# Initial vehicle location and orientation (adjusted spawn pose).
init_loc = chrono.ChVector3d(-110.0, 0.0, 0.6)
init_yaw = 0.0
init_rot = chrono.QuatFromAngleZ(init_yaw)

# Closed-loop control targets.
reference_speed = 12.0     # m/s reference speed input for the speed controller
steering_response_time = 5.0   # s to ramp steering 0 -> target (increased response time)
steering_target = 0.35     # final steering command (-1..+1)

# PID throttle gains (on speed error = reference_speed - measured speed).
pid_kp = 0.35
pid_ki = 0.07
pid_kd = 0.02

# Highway triangle-mesh assets (visual + collision).
highway_vis_mesh = chrono.GetChronoDataFile("synchrono/meshes/Highway_vis.obj")
highway_col_mesh = chrono.GetChronoDataFile("synchrono/meshes/Highway_col.obj")

# === Vehicle === catalog BMW_E90 sedan; wrapper owns its ChSystem (NSC, rigid terrain)
vehicle = veh.BMW_E90()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)   # rigid terrain -> NSC + ChContactMaterialNSC
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetChassisFixed(False)                         # MANDATORY — a fixed chassis never moves
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
vehicle.SetTireStepSize(step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# === System & bodies (created by the veh.BMW_E90 wrapper) ===
system = vehicle.GetSystem()                  # ChSystemNSC owned by the wrapper
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
chassis = vehicle.GetChassisBody()            # cache: main chassis rigid body, reused every step
# spindles: vehicle.GetVehicle().GetSpindlePos(axle, side); joints: suspension/steering inside wrapper
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())   # report total vehicle mass

# Footprint sanity: wheels must rest on (not through) the highway surface near z=0.
veh_obj = vehicle.GetVehicle()
TIRE_RADIUS = 0.33
spindle_world = [veh_obj.GetSpindlePos(a, s)
                 for a in range(veh_obj.GetNumberAxles())
                 for s in (veh.LEFT, veh.RIGHT)]
wheel_bottom_z = min(p.z for p in spindle_world) - TIRE_RADIUS
assert wheel_bottom_z >= -1.0, (
    f"vehicle spawned far below the road: wheel bottom z={wheel_bottom_z:.3f}; "
    f"raise init_loc.z"
)

# === Terrain === rigid highway built from a triangle-mesh patch (visual + collision mesh)
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,          # mesh carries its own world coordinates
    highway_col_mesh,         # collision mesh
    True,                     # connected mesh
    0.0,                      # sweep sphere radius
)
patch.SetColor(chrono.ChColor(0.55, 0.55, 0.6))
terrain.Initialize()

# Overlay the higher-detail visual mesh on the patch ground body.
vis_mesh = chrono.ChTriangleMeshConnected()
vis_mesh = vis_mesh.CreateFromWavefrontFile(highway_vis_mesh, True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(vis_mesh)
vis_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())

# === Driver === plain DriverInputs — closed-loop PID throttle + ramped steering (no keyboard)
driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = 0.0
driver_inputs.m_steering = 0.0
driver_inputs.m_braking = 0.0

# === Visualization === full Irrlicht vehicle scene: window + sky + chase camera + light + grid
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("BMW E90 — highway speed control")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                    # vehicle truths use a directional light
vis.AddGrid(2.0, 2.0, 60, 60,
            chrono.ChCoordsysd(chrono.ChVector3d(init_loc.x, init_loc.y, 0.0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))   # ground reference grid near spawn
vis.AttachVehicle(vehicle.GetVehicle())

# === Main loop === PID speed tracking + steering ramp; full Synchronize/Advance stack
render_steps = math.ceil(render_step_size / step_size)   # precomputed once
realtime_timer = chrono.ChRealtimeStepTimer()

# PID controller state (integral / previous error) — closed-loop on speed error.
pid_integral = 0.0
pid_prev_error = 0.0

os.makedirs("cam", exist_ok=True)                                     # guard against missing output dir

frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()

        if step_number % render_steps == 0:      # throttled rendering
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # PID throttle on speed error (reference_speed - measured chassis speed).
        speed = vehicle.GetVehicle().GetSpeed()
        error = reference_speed - speed
        pid_integral += error * step_size
        derivative = (error - pid_prev_error) / step_size
        pid_prev_error = error
        throttle_cmd = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative
        driver_inputs.m_throttle = max(0.0, min(1.0, throttle_cmd))
        driver_inputs.m_braking = 0.0

        # Steering ramps to its target over the 5 s response time, then holds.
        ramp = min(1.0, time / steering_response_time)
        driver_inputs.m_steering = max(-1.0, min(1.0, steering_target * ramp))


        # Synchronize the full subsystem stack, then advance all of it.
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        terrain.Advance(step_size)
        vehicle.Advance(step_size)          # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1
        realtime_timer.Spin(step_size)      # spin so wall-clock matches sim time
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === flush logs, assemble review video, plot the time series
