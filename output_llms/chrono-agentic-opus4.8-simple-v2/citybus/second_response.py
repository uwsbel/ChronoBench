import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # chassis spawn over the road
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity heading
step_size = 1e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire substep (s)

bus = veh.CityBus()                                                   # catalog city bus wrapper (owns its system)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                  # chassis collision off vs terrain
bus.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
bus.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tire on rigid road
bus.SetTireStepSize(tire_step_size)                                  # tire integration step
bus.Initialize()                                                     # build the vehicle

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh chassis
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)         # mesh wheels
bus.SetTireVisualizationType(veh.VisualizationType_MESH)          # mesh tires

system = bus.GetSystem()                                             # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                 # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid flat terrain on the bus system
patch_mat = chrono.ChContactMaterialNSC()                           # NSC patch material
patch_mat.SetFriction(0.9)                                           # road friction
patch_mat.SetRestitution(0.01)                                       # nearly inelastic road
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # 200x200 m flat patch
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # grey road
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled road texture
terrain.Initialize()                                                 # build terrain contact

# Data-driven driver: scripted (time, steering, throttle, braking) schedule.
driver_data = veh.vector_Entry([
    veh.DataDriverEntry(0.0, 0.0, 0.0, 0.0),                        # t=0.0  : idle
    veh.DataDriverEntry(0.1, 0.0, 1.0, 0.0),                        # t=0.1  : full throttle, straight
    veh.DataDriverEntry(0.5, 0.7, 1.0, 0.0),                        # t=0.5  : full throttle, hard right steer
])
driver = veh.ChDataDriver(bus.GetVehicle(), driver_data)            # pre-recorded input driver
driver.Initialize()                                                  # arm the driver

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle-aware Irrlicht window
vis.SetWindowTitle("City Bus Data Driver")                          # window title
vis.SetWindowSize(1280, 1024)                                        # window pixels
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 13.0, 0.5)        # follow camera behind the bus
vis.Initialize()                                                     # build the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # corner logo
vis.AddSkyBox()                                                      # sky backdrop
vis.AddLightDirectional()                                            # vehicle truths use a directional light
vis.AttachVehicle(bus.GetVehicle())                                 # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                        # 50 fps render cadence
render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame
render_every = render_steps                                          # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # spin to wall-clock
step_number = 0                                                      # physics step counter
sim_end = 10.0                                                       # total sim time (s)
while vis.Run() and bus.GetSystem().GetChTime() < sim_end:
    time = bus.GetSystem().GetChTime()                              # current sim time

    if step_number % render_steps == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                              # scripted inputs at this time

    driver.Synchronize(time)                                        # advance driver schedule
    terrain.Synchronize(time)                                       # advance terrain
    bus.Synchronize(time, driver_inputs, terrain)                  # feed inputs + terrain to vehicle
    vis.Synchronize(time, driver_inputs)                           # HUD picks up inputs


    driver.Advance(step_size)                                       # step driver
    terrain.Advance(step_size)                                      # step terrain
    bus.Advance(step_size)                                          # steps the wrapper-owned system
    vis.Advance(step_size)                                          # step visuals

    step_number += 1                                                # next physics step
    realtime_timer.Spin(step_size)                                  # match wall-clock to sim time
