import math                                                          # render-cadence math
import pychrono.core as chrono                                       # core PyChrono
import pychrono.vehicle as veh                                       # vehicle catalog
import pychrono.irrlicht as chronoirr                                # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                     # integration step (small for stable mesh contact)
init_loc = chrono.ChVector3d(6, -70, 0.5)                            # vehicle start position
init_rot = chrono.QUNIT                                              # no initial rotation

hmmwv = veh.HMMWV_Full()                                             # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision
hmmwv.SetChassisFixed(False)                                        # chassis free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                         # TMEASY tire on rigid road
hmmwv.SetTireStepSize(step_size)                                    # tire integration step
hmmwv.Initialize()                                                 # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)      # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)        # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)         # tire mesh

system = hmmwv.GetSystem()                                          # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # required for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())              # report total vehicle mass

terrain = veh.RigidTerrain(system)                                 # rigid terrain on the shared system

patch_mat = chrono.ChContactMaterialNSC()                          # terrain contact material
patch_mat.SetFriction(0.9)                                         # friction coefficient
patch_mat.SetRestitution(0.01)                                     # restitution

patch = terrain.AddPatch(                                          # single mesh terrain patch
    patch_mat,                                                     # contact material
    chrono.CSYSNORM,                                               # placed at world origin
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),             # collision mesh
    True,                                                          # connected mesh
    0.01,                                                          # contact thickness (sweep sphere radius)
)
terrain.Initialize()                                               # build the terrain

ground = patch.GetGroundBody()                                     # terrain ground body
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(  # load the visual mesh
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()                     # visual triangle-mesh shape
vis_shape.SetMesh(vis_mesh)                                        # attach the loaded mesh
vis_shape.SetName("Highway_vis")                                   # shape name
vis_shape.SetMutable(False)                                        # static visual mesh
ground.AddVisualShape(vis_shape, chrono.ChFramed())               # add visual mesh to ground body

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht window
vis.SetWindowTitle("HMMWV on Highway terrain")                     # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)       # chase camera
vis.Initialize()                                                  # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # logo
vis.AddSkyBox()                                                   # sky box
vis.AddLightDirectional()                                        # directional light (vehicle truth)
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visuals

render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
driver = veh.ChInteractiveDriverIRR(vis)                         # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                  # steering ramp
driver.SetThrottleDelta(render_step_size / 1.0)                  # throttle ramp
driver.SetBrakingDelta(render_step_size / 0.3)                   # braking ramp
driver.Initialize()                                             # build the driver

render_steps = math.ceil(render_step_size / step_size)          # untagged render cadence
realtime_timer = chrono.ChRealtimeStepTimer()                   # real-time pacing
step_number = 0                                                # render-cadence step counter
while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                       # current sim time

    if step_number % render_steps == 0:                        # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                         # current driver inputs

    driver.Synchronize(time)                                   # sync driver
    terrain.Synchronize(time)                                  # sync terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)           # sync vehicle
    vis.Synchronize(time, driver_inputs)                      # sync visuals

    driver.Advance(step_size)                                 # advance driver
    terrain.Advance(step_size)                                # advance terrain
    hmmwv.Advance(step_size)                                  # advance vehicle (steps the system)
    vis.Advance(step_size)                                    # advance visuals


    step_number += 1                                          # advance step counter
    realtime_timer.Spin(step_size)                            # spin to wall-clock

    if time > 12.0:                                           # stop after 12 s
        break
