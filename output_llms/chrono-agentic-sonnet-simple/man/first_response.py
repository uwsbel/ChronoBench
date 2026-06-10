import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())        # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')    # locate vehicle data files

step_size = 5e-4         # simulation time step (s)
render_fps = 50          # render frames per second
terrainLength = 300.0    # terrain patch X size (m)
terrainWidth = 300.0     # terrain patch Y size (m)

# MAN 10t tire radius ~0.629 m; chassis origin ~= spindle height; init_z = tire_radius
init_loc = chrono.ChVector3d(0, 0, 0.75)     # initial vehicle position (wheels rest on z=0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity rotation (heading along +X)

vehicle = veh.MAN_10t()                                         # MAN 10t truck wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)            # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)         # no chassis mesh collision
vehicle.SetChassisFixed(False)                                   # MANDATORY — fixed chassis won't move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot)) # set spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                   # TMEASY tire model (prompt request)
vehicle.SetTireStepSize(step_size)                              # tire integration step
vehicle.Initialize()                                             # build the vehicle

system = vehicle.GetSystem()                                     # vehicle owns the ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())          # truth's literal mass banner

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis visual
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)    # primitives suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)      # primitives steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)              # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)               # mesh tires

terrain = veh.RigidTerrain(system)                              # rigid terrain (flat road)

patch_mat = chrono.ChContactMaterialNSC()                       # NSC contact material for terrain
patch_mat.SetFriction(0.9)                                       # road surface friction
patch_mat.SetRestitution(0.01)                                   # low restitution

patch = terrain.AddPatch(
    patch_mat,
    chrono.CSYSNORM,    # centered at origin
    terrainLength,
    terrainWidth
)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                 # terrain color tint
terrain.Initialize()                                             # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                # vehicle Irrlicht visual system
vis.SetWindowTitle("MAN 10t Truck on Rigid Terrain")            # window title
vis.SetWindowSize(1280, 720)                                     # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 0.5)    # chase camera behind/above vehicle
vis.Initialize()                                                  # FIRST — Irrlicht rule
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # Chrono logo
vis.AddSkyBox()                                                  # skybox for outdoor scene
vis.AddLightDirectional()                                        # directional lighting (vehicle truth)
vis.AttachVehicle(vehicle.GetVehicle())                          # bind vehicle visuals to window

driver = veh.ChInteractiveDriverIRR(vis)                        # real-time keyboard driver (scored-core)
steering_time = 1.0                                              # s to reach max steering
throttle_time = 1.0                                              # s to reach max throttle
braking_time = 0.3                                               # s to reach max brake
driver.SetSteeringDelta(1.0 / (render_fps * steering_time))     # steering rate per render step
driver.SetThrottleDelta(1.0 / (render_fps * throttle_time))     # throttle rate per render step
driver.SetBrakingDelta(1.0 / (render_fps * braking_time))       # braking rate per render step
driver.Initialize()

render_every = max(1, round(1.0 / (render_fps * step_size)))    # physics steps per render frame

realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock sync timer
step_number = 0
while vis.Run():
    time = system.GetChTime()                                    # current simulation time

    if step_number % render_every == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)                                     # update driver state
    terrain.Synchronize(time)                                    # update terrain
    vehicle.Synchronize(time, driver_inputs, terrain)            # update vehicle subsystems
    vis.Synchronize(time, driver_inputs)                         # update visual system HUD


    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    vehicle.Advance(step_size)                                   # advance vehicle + physics
    vis.Advance(step_size)                                       # advance visual system

    step_number += 1
    realtime_timer.Spin(step_size)                               # sync to real time
