import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                     # integration step (s)
tire_step_size = 1e-3                                                # tire substep (s)
target_speed = 6.0                                                   # path-follower cruise speed (m/s)
throttle_value = 0.3                                                 # prompt: constant throttle 0.3
terrainLength = 200.0                                                # prompt: terrain length 100 -> 200
terrainWidth = 200.0                                                 # square terrain so circle fits
path_radius = 40.0                                                   # circular path radius (m)

init_loc = chrono.ChVector3d(path_radius, 0, 0.5)                    # start on the circle, +X side
init_rot = chrono.QuatFromAngleZ(math.pi / 2)                        # face +Y to begin the left turn

hmmwv = veh.HMMWV_Full()                                             # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision needed
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn on the circle
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire on rigid road
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)   # mesh chassis
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

system = hmmwv.GetSystem()                                          # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # flat rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
patch_mat.SetFriction(0.9)                                         # road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # 200x200 patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # sandy color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled texture
terrain.Initialize()                                               # build terrain bodies

start = chrono.ChVector3d(path_radius, 0, 0.5)                     # path start = vehicle start
path = veh.CirclePath(start, path_radius, 10.0, True, 3)           # circular path, left turn, 3 laps

driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), path, "circle_path", target_speed)  # PID path follower
driver.GetSteeringController().SetLookAheadDistance(5.0)           # steering look-ahead (m)
driver.GetSteeringController().SetGains(0.8, 0, 0)                 # steering PID gains (KP, KI, KD)
driver.GetSpeedController().SetGains(0.4, 0, 0)                    # speed controller gains
driver.Initialize()                                               # initialize the driver

ball_mat = chrono.ChVisualMaterial()                              # material for the two path balls
ball_mat.SetDiffuseColor(chrono.ChColor(0.0, 0.8, 0.0))          # green path markers

ball_start = chrono.ChBody()                                       # first ball: start of the circle
ball_start.SetFixed(True)                                          # static marker
ball_start.SetPos(start)                                           # place at the path start
sph_start = chrono.ChVisualShapeSphere(0.3)                        # 0.3 m sphere
sph_start.SetColor(chrono.ChColor(0.0, 0.8, 0.0))                # green
ball_start.AddVisualShape(sph_start, chrono.ChFramed())          # attach visual
system.AddBody(ball_start)                                         # register marker

ball_center = chrono.ChBody()                                      # second ball: center of the circle
ball_center.SetFixed(True)                                         # static marker
ball_center.SetPos(chrono.ChVector3d(0, 0, 0.5))                 # circle center
sph_center = chrono.ChVisualShapeSphere(0.3)                       # 0.3 m sphere
sph_center.SetColor(chrono.ChColor(0.0, 0.8, 0.0))              # green
ball_center.AddVisualShape(sph_center, chrono.ChFramed())        # attach visual
system.AddBody(ball_center)                                        # register marker

sentinel_marker = chrono.ChBody()                                 # sentinel point marker (vehicle ahead point)
sentinel_marker.SetFixed(True)                                    # moved each frame, no dynamics
sph_sent = chrono.ChVisualShapeSphere(0.25)                       # 0.25 m sphere
sph_sent.SetColor(chrono.ChColor(0.0, 0.0, 1.0))               # blue sentinel
sentinel_marker.AddVisualShape(sph_sent, chrono.ChFramed())     # attach visual
system.AddBody(sentinel_marker)                                   # register marker

target_marker = chrono.ChBody()                                  # target point marker (point on the path)
target_marker.SetFixed(True)                                     # moved each frame, no dynamics
sph_tgt = chrono.ChVisualShapeSphere(0.25)                       # 0.25 m sphere
sph_tgt.SetColor(chrono.ChColor(1.0, 0.0, 0.0))              # red target
target_marker.AddVisualShape(sph_tgt, chrono.ChFramed())       # attach visual
system.AddBody(target_marker)                                    # register marker

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle Irrlicht visual system
vis.SetWindowTitle("HMMWV Circular Path Follower")             # window title
vis.SetWindowSize(1280, 1024)                                   # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # follow the chassis
vis.Initialize()                                               # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                # sky box
vis.AddLightDirectional()                                      # directional light (vehicle truths)
vis.AttachVehicle(hmmwv.GetVehicle())                          # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                   # render cadence (s)
render_steps = math.ceil(render_step_size / step_size)         # steps between rendered frames
render_every = render_steps                                    # untagged cadence constant
sim_end = 25.0                                                  # simulation duration (s)


realtime_timer = chrono.ChRealtimeStepTimer()                 # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()                            # current sim time

        driver_inputs = driver.GetInputs()                   # PID steering from the path follower
        driver_inputs.m_throttle = throttle_value            # override: constant throttle 0.3
        driver_inputs.m_braking = 0.0                        # no braking

        sentinel_marker.SetPos(driver.GetSteeringController().GetSentinelLocation())  # show sentinel
        target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())      # show target

        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)                             # advances the wrapper-owned system
        vis.Advance(step_size)

        realtime_timer.Spin(step_size)                       # pace to wall-clock
        if system.GetChTime() >= sim_end:
            break
