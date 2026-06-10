import math                                                         # for render-step rounding
import pychrono.core as chrono                                      # core PyChrono
import pychrono.vehicle as veh                                      # wheeled vehicle catalog
import pychrono.irrlicht as chronoirr                               # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())               # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')           # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                            # chassis spawn (Z above ground)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                        # identity orientation
step_size = 1e-3                                                    # integration step (s)
sim_end = 10.0                                                      # simulation duration (s)

uazbus = veh.UAZBUS()                                               # UAZ bus catalog wrapper
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision mesh
uazbus.SetChassisFixed(False)                                      # chassis must be free to move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # initial pose
uazbus.SetTireType(veh.TireModelType_RIGID)                       # prompt: rigid tire model
uazbus.SetTireStepSize(step_size)                                 # tire integration step
uazbus.Initialize()                                               # build the vehicle

uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)   # chassis mesh
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)     # wheels mesh
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)      # tires mesh

system = uazbus.GetSystem()                                        # take the wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                                # rigid ground under the bus
patch_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch_mat.SetFriction(0.9)                                        # tire-ground friction
patch_mat.SetRestitution(0.01)                                    # near-inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # flat 100x100 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # ground tint
terrain.Initialize()                                              # build terrain

box_mat = chrono.ChContactMaterialNSC()                           # obstacle contact material
box_mat.SetFriction(0.9)                                          # obstacle friction
box = chrono.ChBodyEasyBox(0.5, 5, 0.2, 1000, True, True, box_mat)  # box obstacle (0.5x5x0.2)
box.SetPos(chrono.ChVector3d(5, 0, 0.1))                          # placed ahead of the bus
box.SetFixed(True)                                                # obstacle fixed in place
system.AddBody(box)                                               # add obstacle to the world

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                  # vehicle Irrlicht window
vis.SetWindowTitle("UAZBUS")                                      # window title
vis.SetWindowSize(1280, 1024)                                    # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)      # chase camera on chassis
vis.Initialize()                                                 # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                  # sky box
vis.AddLightDirectional()                                        # directional light (vehicle truth)
vis.AttachVehicle(uazbus.GetVehicle())                           # bind chassis/wheel/tire assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive driver (truth uses this)
driver.SetSteeringDelta(0.02)                                    # steering ramp rate
driver.SetThrottleDelta(0.02)                                    # throttle ramp rate
driver.SetBrakingDelta(0.06)                                     # braking ramp rate
driver.Initialize()                                              # build the driver

render_step_size = 1.0 / 50.0                                    # 50 FPS render cadence
render_steps = math.ceil(render_step_size / step_size)          # physics steps per frame
render_every = render_steps                                     # untagged cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                   # wall-clock pacing
step_number = 0                                                 # render-cadence counter
while vis.Run() and system.GetChTime() < sim_end:
    time = system.GetChTime()                                  # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver.SetThrottle(0.5)                                    # constant forward throttle 0.5
    driver_inputs = driver.GetInputs()                         # current driver inputs

    driver.Synchronize(time)                                   # sync driver
    terrain.Synchronize(time)                                  # sync terrain
    uazbus.Synchronize(time, driver_inputs, terrain)           # sync vehicle with inputs
    vis.Synchronize(time, driver_inputs)                       # sync visualization


    driver.Advance(step_size)                                  # advance driver
    terrain.Advance(step_size)                                 # advance terrain
    uazbus.Advance(step_size)                                  # advance wrapper-owned system
    vis.Advance(step_size)                                     # advance visualization

    step_number += 1                                           # next step
    realtime_timer.Spin(step_size)                             # pace to wall clock
