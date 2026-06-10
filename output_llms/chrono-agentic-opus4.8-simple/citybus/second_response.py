import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                      # integration step (s)
sim_end = 8.0                                                         # total simulated time (s)
init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn (world)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

bus = veh.CityBus()                                                   # catalog city-bus wrapper
bus.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                 # no wrapper chassis collision
bus.SetChassisFixed(False)                                           # MANDATORY — chassis must move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
bus.SetTireType(veh.TireModelType_TMEASY)                            # TMeasy tires on rigid road
bus.SetTireStepSize(step_size)                                       # tire integration step
bus.Initialize()                                                     # build the vehicle subsystems

system = bus.GetSystem()                                             # wrapper owns the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                 # report total vehicle mass

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)           # wheel meshes
bus.SetTireVisualizationType(veh.VisualizationType_MESH)            # tire meshes

terrain = veh.RigidTerrain(system)                                   # flat rigid road
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                           # road friction
patch_mat.SetRestitution(0.01)                                       # near-inelastic
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # road tint
terrain.Initialize()                                                 # build terrain body

driver_data = veh.vector_Entry([                                     # (time, steering, throttle, braking)
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                        # t=0.0: idle
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),                        # t=0.1: full throttle straight
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),                        # t=0.5: full throttle, steer 0.7
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)            # data-driven open-loop driver
driver.Initialize()                                                  # prime the driver schedule

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-aware Irrlicht window
vis.SetWindowTitle("City Bus Data Driver")                          # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 12.0, 0.5)       # chase the chassis
vis.Initialize()                                                     # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # vehicle scenes use directional light
vis.AttachVehicle(bus.GetVehicle())                                 # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                        # 50 FPS target render cadence
render_steps = math.ceil(render_step_size / step_size)             # physics steps per rendered frame
render_every = render_steps                                         # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                       # current sim time

    vis.BeginScene()                                               # render once per frame
    vis.Render()
    vis.EndScene()

    for _ in range(render_every):
        time = system.GetChTime()                                  # step-local time
        driver_inputs = driver.GetInputs()                         # scripted inputs at this time


        driver.Synchronize(time)                                   # advance driver schedule
        terrain.Synchronize(time)                                  # update terrain
        bus.Synchronize(time, driver_inputs, terrain)             # feed inputs + terrain to vehicle
        vis.Synchronize(time, driver_inputs)                      # update HUD/visuals

        driver.Advance(step_size)                                  # step driver
        terrain.Advance(step_size)                                 # step terrain
        bus.Advance(step_size)                                     # step wrapper-owned system
        vis.Advance(step_size)                                     # step visuals

        realtime_timer.Spin(step_size)                             # pace to wall clock
        if system.GetChTime() >= sim_end:
            break
