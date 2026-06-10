import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-90.0, 1.5, 0.5)                         # adjusted initial location on the highway lane
init_yaw = 0.0                                                        # adjusted initial heading (aligned with the road)
init_rot = chrono.QuatFromAngleZ(init_yaw)                            # orientation quaternion about Z

step_size = 1e-3                                                      # decreased simulation step size (finer control)
tire_step_size = step_size                                           # tire integration step
render_step_size = 1.0 / 100.0                                        # decreased render step size (finer control)

vehicle = veh.BMW_E90()                                              # sedan catalog wrapper (BMW E90)
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision shape
vehicle.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # spawn pose on highway
vehicle.SetTireType(veh.TireModelType_TMEASY)                       # TMeasy tire model
vehicle.SetTireStepSize(tire_step_size)                             # tire sub-step
vehicle.Initialize()                                                # build the vehicle subsystems

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # primitive suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)            # mesh tires

system = vehicle.GetSystem()                                         # take wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED, after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())             # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # road friction
patch_mat.SetRestitution(0.01)                                      # near-inelastic contact

patch = terrain.AddPatch(                                            # highway mesh patch
    patch_mat,
    chrono.CSYSNORM,                                                # mesh placed at world origin
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),             # highway collision/visual mesh
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture tiling
terrain.Initialize()                                               # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-specific Irrlicht system
vis.SetWindowTitle("Sedan on Highway")                            # window title
vis.SetWindowSize(1280, 720)                                       # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)  # chase camera over the chassis
vis.Initialize()                                                  # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo overlay
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                         # vehicle truths use a directional light
vis.AttachVehicle(vehicle.GetVehicle())                           # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver bound to the vis
steering_time = 5.0                                              # increased steering response time to 5 s
throttle_time = 1.0                                             # throttle ramp time
braking_time = 0.3                                              # braking ramp time
driver.SetSteeringDelta(render_step_size / steering_time)        # steering rate (5 s to full lock)
driver.SetThrottleDelta(render_step_size / throttle_time)        # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)          # braking rate
driver.Initialize()                                             # initialize the driver

ref_speed = 12.0                                               # reference speed input for the vehicle (m/s)
pid_kp = 0.4                                                   # PID proportional gain on speed error
pid_ki = 0.05                                                  # PID integral gain on speed error
pid_kd = 0.0                                                   # PID derivative gain on speed error
pid_integral = 0.0                                            # accumulated integral term
pid_prev_err = 0.0                                            # previous speed error (for derivative)

render_steps = math.ceil(render_step_size / step_size)        # physics steps per rendered frame
render_every = render_steps                                    # untagged cadence constant
sim_end = 18.0                                                # simulation duration (s)


realtime_timer = chrono.ChRealtimeStepTimer()                # spin to match wall-clock
step_number = 0                                              # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                               # current sim time

    vis.BeginScene()                                        # render once per frame
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        time = system.GetChTime()                          # advance loop sim time

        speed = vehicle.GetVehicle().GetSpeed()            # current forward speed
        err = ref_speed - speed                            # speed error vs reference
        pid_integral += err * step_size                    # integrate the error
        derr = (err - pid_prev_err) / step_size            # error derivative
        pid_prev_err = err                                 # store for next step
        cmd = pid_kp * err + pid_ki * pid_integral + pid_kd * derr   # PID throttle command
        throttle = max(0.0, min(1.0, cmd))                 # clamp throttle to [0,1]

        driver_inputs = driver.GetInputs()                 # interactive steering inputs
        driver_inputs.m_throttle = throttle                # override throttle with PID command
        driver_inputs.m_braking = 0.0                      # no braking


        driver.Synchronize(time)                           # synchronize subsystems
        terrain.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)                          # advance subsystems
        terrain.Advance(step_size)
        vehicle.Advance(step_size)                         # advances the wrapper-owned system
        vis.Advance(step_size)

        step_number += 1                                   # bump step counter
        realtime_timer.Spin(step_size)                     # spin so wall-clock matches sim time
        if system.GetChTime() >= sim_end:
            break
