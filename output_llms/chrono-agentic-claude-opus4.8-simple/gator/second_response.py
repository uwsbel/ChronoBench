import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # bundled data root
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data root

initLoc = chrono.ChVector3d(-10, -2, 0.6)                            # spawn on patch1
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT, no rotation

step_size = 2e-3                                                     # integration step
tire_step_size = 1e-3                                                # tire substep
render_step_size = 1.0 / 50.0                                        # 50 FPS real time

gator = veh.Gator()                                                 # catalog Gator wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                  # rigid-terrain NSC
gator.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
gator.SetChassisFixed(False)                                       # chassis free to move
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # initial pose
gator.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire
gator.SetTireStepSize(tire_step_size)                              # tire integration step
gator.Initialize()                                                 # build the vehicle

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh visualization
gator.SetSuspensionVisualizationType(veh.VisualizationType_MESH)    # mesh visualization
gator.SetSteeringVisualizationType(veh.VisualizationType_MESH)      # mesh visualization
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)         # mesh visualization
gator.SetTireVisualizationType(veh.VisualizationType_MESH)          # mesh visualization

system = gator.GetSystem()                                          # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # REQUIRED for contact

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())               # truth diagnostic banner

terrain = veh.RigidTerrain(system)                                  # multi-patch rigid terrain

# patch 1 — flat tiled box patch
patch1_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch1_mat.SetFriction(0.9)                                        # friction
patch1_mat.SetRestitution(0.01)                                    # restitution
patch1 = terrain.AddPatch(patch1_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT),
                          32, 20)                                  # 32x20 box patch
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                     # tile color
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tile texture

# patch 2 — raised flat concrete box patch
patch2_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch2_mat.SetFriction(0.9)                                        # friction
patch2_mat.SetRestitution(0.01)                                    # restitution
patch2 = terrain.AddPatch(patch2_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT),
                          32, 30)                                  # 32x30 box patch
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))                     # concrete color
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # concrete texture

# patch 3 — bump mesh patch (the obstacle bump)
patch3_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch3_mat.SetFriction(0.9)                                        # friction
patch3_mat.SetRestitution(0.01)                                    # restitution
patch3 = terrain.AddPatch(patch3_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # bump mesh
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                     # bump color
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)  # dirt texture

# patch 4 — heightmap patch for gradability testing
patch4_mat = chrono.ChContactMaterialNSC()                         # NSC contact material
patch4_mat.SetFriction(0.9)                                        # friction
patch4_mat.SetRestitution(0.01)                                    # restitution
patch4 = terrain.AddPatch(patch4_mat,
                          chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"),
                          64.0, 64.0, 0.0, 3.0)                    # heightmap (l,w,hMin,hMax)
patch4.SetColor(chrono.ChColor(0.5, 0.8, 0.5))                     # grass color
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)  # grass texture

terrain.Initialize()                                               # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle Irrlicht window
vis.SetWindowTitle('Gator vehicle')                                # window title
vis.SetWindowSize(1280, 1024)                                      # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.75), 6.0, 0.5)       # chase camera
vis.Initialize()                                                   # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo overlay
vis.AddSkyBox()                                                    # sky box
vis.AddLightDirectional()                                          # vehicle directional light
vis.AttachVehicle(gator.GetVehicle())                             # bind vehicle assets

driver = veh.ChInteractiveDriverIRR(vis)                          # interactive keyboard driver
driver.SetSteeringDelta(render_step_size / 1.0)                   # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                   # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                    # braking rate
driver.Initialize()                                               # build the driver

render_steps = math.ceil(render_step_size / step_size)            # steps per rendered frame
realtime_timer = chrono.ChRealtimeStepTimer()                     # wall-clock pacing
step_number = 0                                                   # physics step counter


while vis.Run():
    time = system.GetChTime()                                    # current sim time

    if step_number % render_steps == 0:                          # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                     # update driver
    terrain.Synchronize(time)                                    # update terrain
    gator.Synchronize(time, driver_inputs, terrain)              # update vehicle
    vis.Synchronize(time, driver_inputs)                         # update visuals

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    gator.Advance(step_size)                                     # advance vehicle (steps system)
    vis.Advance(step_size)                                       # advance visuals

    step_number += 1                                             # next step
    realtime_timer.Spin(step_size)                               # match wall clock
