import math                                                           # render-cadence math
import pychrono.core as chrono                                        # core PyChrono
import pychrono.vehicle as veh                                        # vehicle catalog
import pychrono.irrlicht as chronoirr                                 # Irrlicht renderer

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 2e-3                                                      # integration step (s)
tire_step_size = 1e-3                                                 # tire substep (s)
init_loc = chrono.ChVector3d(-20, 5, 0.5)                            # chassis spawn on patch 1 (X, Y, Z)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # identity orientation

hmmwv = veh.HMMWV_Full()                                              # full HMMWV model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                    # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
hmmwv.SetChassisFixed(False)                                         # MANDATORY — chassis must move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                                # tire substep
hmmwv.Initialize()                                                    # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)     # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)       # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)        # tire mesh

system = hmmwv.GetSystem()                                            # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED contact backend
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid multi-patch terrain

patch1_mat = chrono.ChContactMaterialNSC()                          # patch 1 material
patch1_mat.SetFriction(0.9)                                          # friction
patch1_mat.SetRestitution(0.01)                                     # restitution
patch1 = terrain.AddPatch(                                          # flat box patch 1
    patch1_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(-20, 5, 0), chrono.QUNIT),  # patch 1 position
    32, 20)                                                          # length X, width Y
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                      # tan color
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tiled texture

patch2_mat = chrono.ChContactMaterialNSC()                          # patch 2 material
patch2_mat.SetFriction(0.9)                                          # friction
patch2_mat.SetRestitution(0.01)                                     # restitution
patch2 = terrain.AddPatch(                                          # flat box patch 2
    patch2_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(20, -5, 0.2), chrono.QUNIT),  # patch 2 position
    32, 30)                                                          # length X, width Y
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))                      # reddish color
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # concrete texture

patch3_mat = chrono.ChContactMaterialNSC()                          # patch 3 material
patch3_mat.SetFriction(0.9)                                          # friction
patch3_mat.SetRestitution(0.01)                                     # restitution
patch3 = terrain.AddPatch(                                          # mesh patch 3 (bumpy)
    patch3_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(5, -45, 0), chrono.QUNIT),  # patch 3 position
    veh.GetDataFile("terrain/meshes/bump.obj"))                     # bump mesh
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                      # bluish color

patch4_mat = chrono.ChContactMaterialNSC()                          # patch 4 material
patch4_mat.SetFriction(0.9)                                          # friction
patch4_mat.SetRestitution(0.01)                                     # restitution
patch4 = terrain.AddPatch(                                          # heightmap patch 4 (hills)
    patch4_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),  # patch 4 position
    veh.GetDataFile("terrain/height_maps/test64.bmp"), 64, 64, 0, 4)  # heightmap, len, wid, hMin, hMax
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)  # grass texture

terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht vis
vis.SetWindowTitle("Rigid Multi-Patch Terrain")                     # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # chase camera params
vis.Initialize()                                                     # build device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky box
vis.AddLightDirectional()                                           # directional light (vehicle look)
vis.AttachVehicle(hmmwv.GetVehicle())                               # bind vehicle visuals

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver
render_step_size = 1.0 / 50.0                                       # render cadence (s)
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                   # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                    # braking rate
driver.Initialize()                                                 # init driver

render_every = max(1, round(render_step_size / step_size))         # untagged cadence constant
sim_end = 12.0                                                       # simulation end (s)

realtime_timer = chrono.ChRealtimeStepTimer()                      # real-time pacing
while vis.Run() and system.GetChTime() < sim_end:                  # main loop
    vis.BeginScene()                                                # begin frame
    vis.Render()                                                    # draw scene
    vis.EndScene()                                                  # end frame
    for _ in range(render_every):                                   # inner physics batch
        time = system.GetChTime()                                   # current sim time
        driver_inputs = driver.GetInputs()                          # current driver inputs
        driver.Synchronize(time)                                    # sync driver
        terrain.Synchronize(time)                                   # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)            # sync vehicle
        vis.Synchronize(time, driver_inputs)                       # sync vis
        driver.Advance(step_size)                                   # advance driver
        terrain.Advance(step_size)                                  # advance terrain
        hmmwv.Advance(step_size)                                    # advance vehicle + system
        vis.Advance(step_size)                                      # advance vis
        realtime_timer.Spin(step_size)                             # pace to wall clock
        if system.GetChTime() >= sim_end:                          # stop at end
            break
