import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                # vehicle spawn (X, Y, Z)
init_rot = chrono.QuatFromAngleZ(0)                                    # facing +X, no yaw
step_size = 1e-3                                                       # integration step (s)
tire_step_size = 1e-3                                                  # tire model step (s)

uazbus = veh.UAZBUS()                                                  # UAZ bus catalog wrapper
uazbus.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
uazbus.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
uazbus.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
uazbus.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))       # initial pose
uazbus.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tire on rigid road
uazbus.SetTireStepSize(tire_step_size)                                # tire integration step
uazbus.Initialize()                                                   # build the vehicle

uazbus.SetChassisVisualizationType(veh.VisualizationType_MESH)    # chassis mesh
uazbus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension links
uazbus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering links
uazbus.SetWheelVisualizationType(veh.VisualizationType_MESH)      # wheel mesh
uazbus.SetTireVisualizationType(veh.VisualizationType_MESH)       # tire mesh

system = uazbus.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", uazbus.GetVehicle().GetMass())               # report total vehicle mass

terrain = veh.RigidTerrain(system)                                    # rigid ground
patch_mat = chrono.ChContactMaterialNSC()                             # NSC contact material
patch_mat.SetFriction(0.9)                                            # friction coefficient
patch_mat.SetRestitution(0.01)                                        # restitution (bounciness)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200.0, 200.0)   # flat 200x200 m patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # patch color
terrain.Initialize()                                                  # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                      # vehicle Irrlicht window
vis.SetWindowTitle("UAZBUS")                                          # window title
vis.SetWindowSize(1280, 1024)                                         # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 6.0, 0.5)          # chase camera on chassis
vis.Initialize()                                                      # create the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))     # logo
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                            # directional light (vehicle truth)
vis.AttachVehicle(uazbus.GetVehicle())                               # bind vehicle visuals

driver = veh.ChInteractiveDriverIRR(vis)                              # interactive driver bound to vis
render_step_size = 1.0 / 50.0                                         # 50 FPS render cadence
driver.SetSteeringDelta(render_step_size / 1.0)                      # 1 s to full steering
driver.SetThrottleDelta(render_step_size / 1.0)                     # 1 s to full throttle
driver.SetBrakingDelta(render_step_size / 0.3)                      # 0.3 s to full brake
driver.Initialize()                                                  # build the driver

render_steps = math.ceil(render_step_size / step_size)               # physics steps per frame
realtime_timer = chrono.ChRealtimeStepTimer()                        # wall-clock pacing
step_number = 0                                                       # physics step counter

while vis.Run():
    time = system.GetChTime()                                        # current sim time

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver inputs

    driver.Synchronize(time)                                         # update driver
    terrain.Synchronize(time)                                        # update terrain
    uazbus.Synchronize(time, driver_inputs, terrain)                # update vehicle
    vis.Synchronize(time, driver_inputs)                            # update visualization

    driver.Advance(step_size)                                        # advance driver
    terrain.Advance(step_size)                                       # advance terrain
    uazbus.Advance(step_size)                                        # advance vehicle (steps the system)
    vis.Advance(step_size)                                           # advance visualization

    step_number += 1                                                 # next step
    realtime_timer.Spin(step_size)                                   # match wall-clock
