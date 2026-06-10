import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                   # simulation time step (s)
render_fps = 25.0                  # render frames per second
sim_end = 30.0                     # simulation end time (s)
terrainLength = 200.0              # terrain length (m) — increased from 100
terrainWidth = 200.0               # terrain width (m)
path_radius = 40.0                 # circular path radius (m)
target_speed = 8.0                 # target vehicle speed (m/s)
throttle_val = 0.3                 # constant throttle value

# Initial vehicle position (offset so circle fits in terrain)
initLoc = chrono.ChVector3d(path_radius, 0, 0.5)   # start on circle
initRot = chrono.QuatFromAngleZ(math.pi / 2)        # facing tangent to circle

hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)           # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)
hmmwv.SetChassisFixed(False)                                  # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
hmmwv.SetTireType(veh.TireModelType_TMEASY)                  # TMEASY for good traction
hmmwv.SetTireStepSize(step_size)
hmmwv.Initialize()

system = hmmwv.GetSystem()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # truth's vehicle mass banner

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

# Rigid terrain — 200 m x 200 m flat patch
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrainLength, terrainWidth)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Circular path centered at origin with radius path_radius
path_start = chrono.ChVector3d(path_radius, 0, 0.5)          # start at (radius, 0)
path = veh.CirclePath(path_start, path_radius, 5.0, True, 3)  # 3 laps, left turn

# Path visualization: two spheres marking entry and the opposite point on the circle
ball1_body = chrono.ChBody()
ball1_body.SetFixed(True)
ball1_body.SetPos(chrono.ChVector3d(path_radius, 0, 0.5))     # start of path
sphere1 = chrono.ChVisualShapeSphere(0.5)
sphere1.SetColor(chrono.ChColor(0.0, 0.8, 0.0))              # green ball 1
ball1_body.AddVisualShape(sphere1, chrono.ChFramed())
system.AddBody(ball1_body)

ball2_body = chrono.ChBody()
ball2_body.SetFixed(True)
ball2_body.SetPos(chrono.ChVector3d(-path_radius, 0, 0.5))    # opposite side of circle
sphere2 = chrono.ChVisualShapeSphere(0.5)
sphere2.SetColor(chrono.ChColor(0.0, 0.8, 0.0))              # green ball 2
ball2_body.AddVisualShape(sphere2, chrono.ChFramed())
system.AddBody(ball2_body)

# Path follower driver: constant throttle + PID steering
driver = veh.ChPathFollowerDriver(hmmwv.GetVehicle(), path, "circle_path", target_speed)
driver.GetSteeringController().SetLookAheadDistance(5.0)       # look-ahead for PID
driver.GetSteeringController().SetGains(0.8, 0.0, 0.0)        # KP, KI, KD
driver.GetSpeedController().SetGains(0.4, 0.0, 0.0)
driver.Initialize()

# Sentinel marker (red sphere) — updated each frame to steering controller sentinel
sentinel_marker = chrono.ChBody()
sentinel_marker.SetFixed(True)
sentinel_sphere = chrono.ChVisualShapeSphere(0.1)
sentinel_sphere.SetColor(chrono.ChColor(1.0, 0.0, 0.0))      # red = sentinel
sentinel_marker.AddVisualShape(sentinel_sphere, chrono.ChFramed())
system.AddBody(sentinel_marker)

# Target marker (blue sphere) — updated each frame to steering controller target
target_marker = chrono.ChBody()
target_marker.SetFixed(True)
target_sphere = chrono.ChVisualShapeSphere(0.1)
target_sphere.SetColor(chrono.ChColor(0.0, 0.0, 1.0))        # blue = target
target_marker.AddVisualShape(target_sphere, chrono.ChFramed())
system.AddBody(target_marker)

# Irrlicht vehicle visualization — Initialize first, then add scene elements
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("HMMWV Circular Path Follower")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)  # chase camera
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()
vis.AttachVehicle(hmmwv.GetVehicle())

render_every = max(1, round(1.0 / (render_fps * step_size)))  # render cadence (untagged)

realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
while vis.Run():
    time = system.GetChTime()

    if step_number % render_every == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = throttle_val                    # constant throttle 0.3

    # Update sentinel and target markers each frame
    sentinel_marker.SetPos(driver.GetSteeringController().GetSentinelLocation())
    target_marker.SetPos(driver.GetSteeringController().GetTargetLocation())

    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)

    if time >= sim_end:
        break
