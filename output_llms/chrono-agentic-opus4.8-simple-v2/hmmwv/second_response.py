import math                                                         # path geometry / angles
import os                                                           # review-only env flag
import pychrono.core as chrono                                      # core PyChrono types
import pychrono.vehicle as veh                                      # wheeled-vehicle catalog + drivers
import pychrono.irrlicht as chronoirr                               # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())               # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

step_size = 2e-3                                                    # integration step (s)
tire_step_size = 1e-3                                               # tire substep (s)
sim_end = 30.0                                                      # simulation duration (s)

terrainLength = 200.0                                               # X extent (raised 100 -> 200 to fit the circle)
terrainWidth = 200.0                                                # Y extent
init_loc = chrono.ChVector3d(-20, 0, 0.5)                           # chassis spawn (on the circular path start)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                        # identity heading (+X)

hmmwv = veh.HMMWV_Full()                                            # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                 # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)             # no chassis collision geometry
hmmwv.SetChassisFixed(False)                                       # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))     # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                             # tire integration substep
hmmwv.Initialize()                                                 # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)  # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension prims
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering prims
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)    # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)     # tire mesh

system = hmmwv.GetSystem()                                         # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                                # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch_mat.SetFriction(0.9)                                         # tire-road friction
patch_mat.SetRestitution(0.01)                                     # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)  # flat patch 200x200
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # sandy color
terrain.Initialize()                                               # finalize terrain

path_radius = 20.0                                                 # circular path radius (m)
path_run = 5.0                                                     # straight run-in before the circle (m)
path_start = chrono.ChVector3d(-path_radius, 0, 0.5)              # path start at left of the circle
path = veh.CirclePath(path_start, path_radius, path_run, True, 5) # 20 m radius, left turn, 5 laps

target_speed = 8.0                                                 # cruise speed the path follower aims for (m/s)
driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), path, "circle_path", target_speed)  # PID path follower
driver.GetSteeringController().SetLookAheadDistance(5.0)          # sentinel look-ahead (m)
driver.GetSteeringController().SetGains(0.8, 0, 0)               # PID steering gains KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0, 0)                  # speed controller gains (unused: throttle is fixed)
driver.Initialize()                                               # build the controllers

const_throttle = 0.3                                              # constant throttle requested by the prompt

path_line = chrono.ChLineBezier(path)                             # geometric line of the bezier path
path_asset = chrono.ChVisualShapeLine()                           # visual line shape for the path
path_asset.SetLineGeometry(path_line)                            # bind the bezier geometry
path_asset.SetColor(chrono.ChColor(0.0, 0.8, 0.0))              # green path line
path_body = chrono.ChBody()                                       # carrier body for the path visual
path_body.SetFixed(True)                                          # path is static
path_body.AddVisualShape(path_asset, chrono.ChFramed())          # attach the line visual
system.AddBody(path_body)                                         # register the path body

ball_a = chrono.ChBody()                                          # first path-marker ball
ball_a.SetFixed(True)                                             # static marker
ball_a.SetPos(path_start)                                         # at the path start
sphere_a = chrono.ChVisualShapeSphere(0.4)                        # visualize the path with a ball
sphere_a.SetColor(chrono.ChColor(0.0, 0.8, 0.0))                # green
ball_a.AddVisualShape(sphere_a, chrono.ChFramed())               # attach
system.AddBody(ball_a)                                            # register
ball_b = chrono.ChBody()                                          # second path-marker ball
ball_b.SetFixed(True)                                             # static marker
ball_b.SetPos(chrono.ChVector3d(path_radius, 0, 0.5))           # opposite side of the circle
sphere_b = chrono.ChVisualShapeSphere(0.4)                        # second ball
sphere_b.SetColor(chrono.ChColor(0.0, 0.8, 0.0))                # green
ball_b.AddVisualShape(sphere_b, chrono.ChFramed())               # attach
system.AddBody(ball_b)                                            # register

sentinel_marker = chrono.ChBody()                                # follows the controller sentinel point
sentinel_marker.SetFixed(True)                                   # kinematically positioned each frame
sentinel_sphere = chrono.ChVisualShapeSphere(0.25)              # sentinel sphere
sentinel_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))        # red sentinel
sentinel_marker.AddVisualShape(sentinel_sphere, chrono.ChFramed())  # attach
system.AddBody(sentinel_marker)                                  # register
target_marker = chrono.ChBody()                                 # follows the controller target point
target_marker.SetFixed(True)                                    # kinematically positioned each frame
target_sphere = chrono.ChVisualShapeSphere(0.25)               # target sphere
target_sphere.SetColor(chrono.ChColor(1.0, 1.0, 0.0))         # yellow target
target_marker.AddVisualShape(target_sphere, chrono.ChFramed())  # attach
system.AddBody(target_marker)                                   # register

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                 # vehicle-aware Irrlicht window
vis.SetWindowTitle("HMMWV Circular Path Follower")              # window title
vis.SetWindowSize(1280, 1024)                                    # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)    # chase cam track point / dist / height
vis.Initialize()                                                 # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                  # sky
vis.AddLightDirectional()                                       # directional light (vehicle truth style)
vis.AttachVehicle(hmmwv.GetVehicle())                          # bind vehicle visual assets

render_step_size = 1.0 / 50.0                                   # 50 fps render cadence
render_every = max(1, round(render_step_size / step_size))     # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                  # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:              # real-time loop
    vis.BeginScene()                                           # start frame
    vis.Render()                                               # draw scene
    vis.EndScene()                                             # end frame
    for _ in range(render_every):                              # physics substeps between frames
        time = system.GetChTime()                              # current sim time

        sentinel_marker.SetPos(driver.GetSteeringController().GetSentinelLocation())  # move sentinel ball
        target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())      # move target ball

        driver_inputs = driver.GetInputs()                     # PID steering inputs
        driver_inputs.m_throttle = const_throttle              # override with constant throttle 0.3
        driver_inputs.m_braking = 0.0                          # no braking

        driver.Synchronize(time)                               # advance the path controllers
        terrain.Synchronize(time)                              # terrain bookkeeping
        hmmwv.Synchronize(time, driver_inputs, terrain)       # feed inputs to the vehicle
        vis.Synchronize(time, driver_inputs)                  # HUD picks up inputs

        driver.Advance(step_size)                              # step driver
        terrain.Advance(step_size)                             # step terrain
        hmmwv.Advance(step_size)                               # step the wrapper-owned system
        vis.Advance(step_size)                                 # step visualization

        realtime_timer.Spin(step_size)                         # pace to wall clock
        if system.GetChTime() >= sim_end:                      # stop at the end
            break
