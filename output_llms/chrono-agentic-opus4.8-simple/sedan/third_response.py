import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 1e-3                                                      # finer simulation step size
tire_step_size = 1e-3                                                 # tire integration step
render_step_size = 1.0 / 100.0                                        # finer render step size

init_loc = chrono.ChVector3d(-110.0, 0.0, 0.6)                        # adjusted initial location on the highway
init_rot = chrono.QuatFromAngleZ(0.0)                                 # adjusted initial heading (along +X)

sedan = veh.BMW_E90()                                                 # BMW E90 sedan catalog vehicle
sedan.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid mesh terrain
sedan.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision against road
sedan.SetChassisFixed(False)                                         # chassis must be free to move
sedan.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # set spawn pose
sedan.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire on rigid terrain
sedan.SetTireStepSize(tire_step_size)                               # tire solver step
sedan.Initialize()                                                  # build the vehicle

sedan.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
sedan.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
sedan.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
sedan.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = sedan.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
print("VEHICLE MASS: ", sedan.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                  # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                          # road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic road
patch = terrain.AddPatch(                                          # highway mesh patch
    patch_mat,
    chrono.CSYSNORM,                                              # no offset / rotation
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),            # collision + visual highway mesh
)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)  # road texture
terrain.Initialize()                                              # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht visual system
vis.SetWindowTitle("Sedan on highway with PID speed control")     # window title
vis.SetWindowSize(1280, 1024)                                    # window size
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)  # chase camera behind chassis
vis.Initialize()                                                # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                 # sky box
vis.AddLightDirectional()                                      # vehicle scenes use a directional light
vis.AttachVehicle(sedan.GetVehicle())                          # bind chassis/wheel/tire visuals

driver_inputs = veh.DriverInputs()                            # closed-loop driver-inputs struct
driver_inputs.m_throttle = 0.0                               # initial throttle
driver_inputs.m_steering = 0.0                               # initial steering
driver_inputs.m_braking = 0.0                                # initial braking

sim_end = 14.0                                              # total simulated time (s)
ref_speed = 12.0                                             # reference speed input (m/s)
steering_time = 5.0                                         # increased steering response time (s)
steering_target = 0.2                                       # commanded steady steering

kp = 0.4                                                    # PID proportional gain
ki = 0.05                                                   # PID integral gain
kd = 0.01                                                   # PID derivative gain
integral_err = 0.0                                          # accumulated speed error
prev_err = 0.0                                              # previous speed error for derivative

render_steps = math.ceil(render_step_size / step_size)       # untagged render cadence

realtime_timer = chrono.ChRealtimeStepTimer()                     # real-time pacing
step_number = 0                                                  # physics step counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                   # current sim time

    if step_number % render_steps == 0:                         # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    speed = sedan.GetVehicle().GetSpeed()                       # current forward speed
    err = ref_speed - speed                                     # speed error vs reference
    integral_err += err * step_size                             # integrate error
    deriv_err = (err - prev_err) / step_size                    # error derivative
    prev_err = err                                              # store for next step
    throttle = kp * err + ki * integral_err + kd * deriv_err    # PID throttle command
    throttle = max(0.0, min(1.0, throttle))                     # clamp throttle to [0, 1]
    driver_inputs.m_throttle = throttle                        # apply PID throttle
    steering = steering_target * min(1.0, time / steering_time)  # ramp steering over 5 s
    driver_inputs.m_steering = steering                        # apply ramped steering


    terrain.Synchronize(time)
    sedan.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    terrain.Advance(step_size)
    sedan.Advance(step_size)                                    # advances the wrapper-owned system
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                             # match wall-clock to sim time
