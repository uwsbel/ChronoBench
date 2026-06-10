import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate Chrono data
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data

# Vehicle parameters
initLoc = chrono.ChVector3d(-15, 0, 0.5)                             # initial position (turn2: -15,0,0.5)
initRot = chrono.QuatFromAngleZ(0)                                   # facing +X direction

step_size = 1e-3                                                     # physics step size
tire_step_size = 1e-3                                                 # tire integration step
sim_end = 30.0                                                       # total simulation time

# Create Kraz tractor-trailer
vehicle = veh.Kraz()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision
vehicle.SetChassisFixed(False)                                       # allow chassis to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))       # set start pose
vehicle.SetTireStepSize(tire_step_size)                              # tire integrator step
vehicle.Initialize()

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES, veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH, veh.VisualizationType_MESH)

system = vehicle.GetSystem()                                         # get the vehicle-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", vehicle.GetTractor().GetMass())              # vehicle mass diagnostic

# Rigid terrain
terrain = veh.RigidTerrain(system)                                   # terrain attached to vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # low restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300.0, 100.0)  # large flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Vehicle visualization (Irrlicht via wheeled vehicle visual system)
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Kraz Tractor-Trailer Double Lane Change")        # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(3, 0, 2.1), 25.0, 10.5)        # turn2: trackPt=(3,0,2.1), dist=25.0, height=10.5
vis.Initialize()                                                     # create Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # Chrono logo
vis.AddSkyBox()                                                      # sky background
vis.AddLightDirectional()                                            # directional light (vehicle truth style)
vis.AttachVehicle(vehicle.GetTractor())                              # Kraz: attach via GetTractor(), not GetVehicle()

# Interactive driver (scored-core default) — double lane change maneuver scripted in loop
driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver attached to vis
steering_time = 1.0                                                  # time to reach max steering
throttle_time = 1.0                                                  # time to reach max throttle
braking_time = 0.3                                                   # time to reach max braking
driver.SetSteeringDelta(1.0 / (50.0 * steering_time))               # steering delta per render step
driver.SetThrottleDelta(1.0 / (50.0 * throttle_time))               # throttle delta per render step
driver.SetBrakingDelta(1.0 / (50.0 * braking_time))                 # braking delta per render step
driver.Initialize()

# Render and simulation parameters
render_fps = 50.0                                                    # target render frames per second
render_step_size = 1.0 / render_fps                                  # time between rendered frames
render_steps = math.ceil(render_step_size / step_size)               # physics steps per render frame
render_every = render_steps                                          # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time pacing


step_number = 0
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                        # current sim time

    if step_number % render_steps == 0:                              # throttled render
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # get driver inputs

    # Double lane change maneuver: override driver inputs directly per simulation time
    dlc_inputs = veh.DriverInputs()                                  # scripted double lane change (scored core)
    if time < 3.0:                                                   # phase 1: straight acceleration
        dlc_inputs.m_throttle = 0.6
        dlc_inputs.m_steering = 0.0
        dlc_inputs.m_braking = 0.0
    elif time < 4.0:                                                 # phase 2: steer left (first lane change)
        dlc_inputs.m_throttle = 0.5
        dlc_inputs.m_steering = -0.5
        dlc_inputs.m_braking = 0.0
    elif time < 6.0:                                                 # phase 3: straight in new lane
        dlc_inputs.m_throttle = 0.5
        dlc_inputs.m_steering = 0.0
        dlc_inputs.m_braking = 0.0
    elif time < 7.0:                                                 # phase 4: steer right (second lane change)
        dlc_inputs.m_throttle = 0.4
        dlc_inputs.m_steering = 0.5
        dlc_inputs.m_braking = 0.0
    else:                                                            # phase 5: straight in original lane
        dlc_inputs.m_throttle = 0.4
        dlc_inputs.m_steering = 0.0
        dlc_inputs.m_braking = 0.0


    driver.Synchronize(time)                                         # update interactive driver state
    terrain.Synchronize(time)                                        # update terrain
    vehicle.Synchronize(time, dlc_inputs, terrain)                   # update vehicle with scripted inputs
    vis.Synchronize(time, dlc_inputs)                                # update visualization

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    vehicle.Advance(step_size)                                       # advance vehicle (steps the system)
    vis.Advance(step_size)                                           # advance vis

    step_number += 1
    realtime_timer.Spin(step_size)                                   # real-time pacing
