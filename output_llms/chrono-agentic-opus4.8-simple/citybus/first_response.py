import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core Chrono API
import pychrono.vehicle as veh                                        # vehicle catalog
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                              # vehicle spawn location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # vehicle spawn orientation (identity)
step_size = 1e-3                                                      # integration step (s)

bus = veh.CityBus()                                                   # CityBus catalog wrapper (owns its system)
bus.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
bus.SetChassisCollisionType(veh.CollisionType_NONE)                 # no chassis collision mesh
bus.SetChassisFixed(False)                                           # chassis must move
bus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))         # initial pose
bus.SetTireType(veh.TireModelType_TMEASY)                           # TMEASY tire model for rigid road
bus.SetTireStepSize(step_size)                                       # tire integration step
bus.Initialize()                                                     # build the vehicle

bus.SetChassisVisualizationType(veh.VisualizationType_MESH)                # chassis as mesh
bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)      # suspension as primitives
bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)        # steering as primitives
bus.SetWheelVisualizationType(veh.VisualizationType_MESH)                  # wheels as mesh
bus.SetTireVisualizationType(veh.VisualizationType_MESH)                   # tires as mesh

system = bus.GetSystem()                                             # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # required for contact
print("VEHICLE MASS: ", bus.GetVehicle().GetMass())                 # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid terrain on the bus system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC terrain material
patch_mat.SetFriction(0.9)                                           # road friction
patch_mat.SetRestitution(0.01)                                       # low bounce
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)  # flat 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # custom road texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # road tint
terrain.Initialize()                                                 # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-aware Irrlicht system
vis.SetWindowTitle("CityBus on Rigid Terrain")                       # window title
vis.SetWindowSize(1280, 1024)                                        # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 2.0), 12.0, 0.8)         # camera follows the bus
vis.Initialize()                                                     # create the device (FIRST)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # directional light (vehicle truth)
vis.AttachVehicle(bus.GetVehicle())                                 # bind chassis/wheel/tire visuals

render_step_size = 1.0 / 50.0                                        # render at 50 fps
render_steps = math.ceil(render_step_size / step_size)              # physics steps per rendered frame

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                    # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                     # braking ramp rate
driver.Initialize()                                                 # build the driver

render_every = render_steps                                          # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # wall-clock pacing
step_number = 0                                                      # physics step counter
while vis.Run():
    time = system.GetChTime()                                       # current sim time

    if step_number % render_steps == 0:                            # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                             # current driver inputs

    driver.Synchronize(time)                                        # update driver
    terrain.Synchronize(time)                                       # update terrain
    bus.Synchronize(time, driver_inputs, terrain)                  # update vehicle with inputs
    vis.Synchronize(time, driver_inputs)                          # update visualization

    driver.Advance(step_size)                                      # advance driver
    terrain.Advance(step_size)                                     # advance terrain
    bus.Advance(step_size)                                         # advance vehicle (steps the system)
    vis.Advance(step_size)                                         # advance visualization

    step_number += 1                                               # next step
    realtime_timer.Spin(step_size)                                 # pace to wall clock
