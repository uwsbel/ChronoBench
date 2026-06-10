import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

init_loc = chrono.ChVector3d(-15, 0, 1.2)                              # raised spawn for the hill
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT — no initial rotation
step_size = 1e-3                                                       # integration step
tire_step_size = step_size                                            # tire substep matches main step

hmmwv = veh.HMMWV_Full()                                               # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                     # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision shape
hmmwv.SetChassisFixed(False)                                           # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # initial pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                                  # tire model substep
hmmwv.Initialize()                                                     # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)    # chassis as primitives
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES) # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)            # wheels as mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)             # tires as mesh

system = hmmwv.GetSystem()                                             # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED for terrain contact
system.GetSolver().AsIterative().SetMaxIterations(150)                 # more NSC iterations for stable mesh contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                  # report total vehicle mass

terrain = veh.RigidTerrain(system)                                     # rigid terrain
patch_mat = chrono.ChContactMaterialNSC()                              # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                             # patch friction
patch_mat.SetRestitution(0.01)                                         # patch restitution
patch = terrain.AddPatch(                                              # single heightmap patch (a hill)
    patch_mat,
    chrono.CSYSNORM,                                                   # centered at origin, no rotation
    veh.GetDataFile("terrain/height_maps/bump64.bmp"),                 # heightmap image
    40, 40,                                                            # length, width (m)
    -1, 1,                                                             # hMin, hMax (m)
    False,                                                             # height-field collision (stable, no triangle-soup fall-through)
    0.01,                                                              # sweep-sphere radius
)
patch.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture, UV tiling
terrain.Initialize()                                                   # build the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle-specific Irrlicht window
vis.SetWindowTitle("SCM Hill Demo")                                    # window title
vis.SetWindowSize(1280, 1024)                                          # window size (px)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.71), 6.0, 0.5)        # chase camera track point/dist/height
vis.Initialize()                                                       # build the device FIRST
vis.AddLogo()                                                          # pychrono logo
vis.AddLightDirectional()                                             # single directional light (vehicle style)
vis.AddSkyBox()                                                        # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                                  # bind the vehicle to the visual system

driver = veh.ChInteractiveDriverIRR(vis)                               # interactive keyboard driver
render_step_size = 1.0 / 20.0                                          # render cadence for the hill demo
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering ramp rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle ramp rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking ramp rate
driver.Initialize()                                                    # build the driver

render_steps = math.ceil(render_step_size / step_size)                # physics steps between rendered frames
realtime_timer = chrono.ChRealtimeStepTimer()                          # real-time pacing
step_number = 0                                                        # physics step counter

while vis.Run():                                                       # main real-time loop
    time = hmmwv.GetSystem().GetChTime()                              # current sim time

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                               # current driver commands

    driver.Synchronize(time)                                          # advance driver state
    terrain.Synchronize(time)                                         # advance terrain state
    hmmwv.Synchronize(time, driver_inputs, terrain)                  # feed driver + terrain to the vehicle
    vis.Synchronize(time, driver_inputs)                             # update the visual system

    driver.Advance(step_size)                                        # step driver
    terrain.Advance(step_size)                                       # step terrain
    hmmwv.Advance(step_size)                                         # step the wrapper-owned system
    vis.Advance(step_size)                                           # step the visual system

    step_number += 1                                                 # next step
    realtime_timer.Spin(step_size)                                   # pace to wall clock
