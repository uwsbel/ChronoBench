import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

# Vehicle initial position and orientation
init_loc = chrono.ChVector3d(0, 0, 0.5)                             # start slightly above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # facing forward (identity)

step_size = 1.0 / 500.0                                              # 500 Hz physics
render_fps = 50.0                                                     # 50 fps rendering (as specified)
sim_end = 20.0                                                        # simulate for 20 s

# Create CityBus vehicle
bus = veh.CityBus()
bus.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision mesh
bus.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
bus.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tires for rigid terrain
bus.SetTireStepSize(step_size)
bus.Initialize()

system = bus.GetSystem()                                              # get the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                  # report total vehicle mass

# Use mesh vis for chassis, primitives for suspension/steering/wheels, mesh for tires
bus.SetChassisVisualizationType(veh.VisualizationType_MESH)          # mesh chassis
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # primitives for suspension
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitives for steering
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)            # mesh wheels
bus.SetTireVisualizationType(veh.VisualizationType_MESH)             # mesh tires

# Create rigid terrain with custom texture
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()                            # NSC matches bus contact method
patch_mat.SetFriction(0.9)                                           # high friction road
patch_mat.SetRestitution(0.01)                                       # nearly inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 300.0, 300.0)  # large flat road patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sand/road color
terrain.Initialize()

# Vehicle-specific Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("CityBus on Rigid Terrain")
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 15.0, 0.5)         # follow bus from behind, 15m back
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddLightDirectional()                                             # vehicle truths use directional light
vis.AttachVehicle(bus.GetVehicle())

# Interactive driver — allows steering, throttle, braking via keyboard
driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0                                                  # s to reach max steering
throttle_time = 1.0                                                  # s to reach max throttle
braking_time = 0.3                                                   # s to reach max brake

render_step_size = 1.0 / render_fps                                  # time per render frame
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

render_every = max(1, round(render_step_size / step_size))           # physics steps per render frame


realtime_timer = chrono.ChRealtimeStepTimer()                        # keep sim at real-time pace
step_number = 0

while vis.Run() and system.GetChTime() < sim_end:
    sim_time = system.GetChTime()

    if step_number % render_every == 0:                              # throttled at 50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()


    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    bus.Synchronize(sim_time, driver_inputs, terrain)                # synchronize vehicle subsystems
    vis.Synchronize(sim_time, driver_inputs)


    driver.Advance(step_size)
    terrain.Advance(step_size)
    bus.Advance(step_size)                                           # advances wrapper-owned system
    vis.Advance(step_size)

    step_number += 1
    realtime_timer.Spin(step_size)                                   # maintain real-time pace
