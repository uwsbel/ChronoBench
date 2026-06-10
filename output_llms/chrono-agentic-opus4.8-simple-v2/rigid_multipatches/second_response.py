import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())              # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')          # locate vehicle data files

step_size = 2e-3                                                   # integration step (s)
tire_step_size = 1e-3                                              # tire substep (s)
init_loc = chrono.ChVector3d(6, -70, 0.5)                         # HMMWV start position on the highway
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                       # no initial yaw

hmmwv = veh.HMMWV_Full()                                          # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)            # no chassis collision shape
hmmwv.SetChassisFixed(False)                                      # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))    # spawn pose
hmmwv.SetTireType(veh.TireModelType_TMEASY)                       # TMEASY tires for rigid road
hmmwv.SetTireStepSize(tire_step_size)                            # tire model substep
hmmwv.Initialize()                                               # build the vehicle subsystems

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis mesh
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension primitives
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering primitives
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # wheel mesh
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # tire mesh

system = hmmwv.GetSystem()                                       # wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())            # report total vehicle mass

terrain = veh.RigidTerrain(system)                              # rigid terrain attached to the vehicle system

patch_mat = chrono.ChContactMaterialNSC()                        # contact material for the highway patch
patch_mat.SetFriction(0.9)                                       # tire-road friction
patch_mat.SetRestitution(0.01)                                   # near-inelastic contacts

# single mesh terrain patch from the Highway collision mesh (replaces the former multiple patches)
patch = terrain.AddPatch(
    patch_mat,                                                   # NSC contact material
    chrono.CSYSNORM,                                             # mesh placed at world origin
    veh.GetDataFile("terrain/meshes/Highway_col.obj"),          # collision mesh file
    True,                                                        # connected mesh
    0.01,                                                        # contact material thickness (sweep sphere radius)
)

# visual mesh for the terrain ground body, loaded from the Highway visual mesh
vis_mesh = chrono.ChTriangleMeshConnected()                      # empty mesh container
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), False, True)  # load OBJ
vis_shape = chrono.ChVisualShapeTriangleMesh()                  # triangle-mesh visual shape
vis_shape.SetMesh(vis_mesh)                                      # attach the loaded mesh
vis_shape.SetName("Highway_vis")                                 # shape name
vis_shape.SetMutable(False)                                      # static visual geometry
patch.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())  # add visual mesh to the terrain ground body

terrain.Initialize()                                            # finalize terrain bodies/contacts

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()               # vehicle-aware Irrlicht visual system
vis.SetWindowTitle("HMMWV on Highway Mesh Terrain")            # window title
vis.SetWindowSize(1280, 1024)                                  # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)   # chase camera behind the chassis
vis.Initialize()                                              # create the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))  # PyChrono logo
vis.AddSkyBox()                                               # sky box
vis.AddLightDirectional()                                    # directional light (vehicle scenes)
vis.AttachVehicle(hmmwv.GetVehicle())                        # bind chassis/wheel/tire visuals

driver = veh.ChInteractiveDriverIRR(vis)                     # interactive driver bound to the vis
steering_time = 1.0                                          # s to go 0 -> +1 steering
throttle_time = 1.0                                          # s to go 0 -> +1 throttle
braking_time = 0.3                                           # s to go 0 -> +1 brake
render_step_size = 1.0 / 50.0                                # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)   # steering ramp rate
driver.SetThrottleDelta(render_step_size / throttle_time)   # throttle ramp rate
driver.SetBrakingDelta(render_step_size / braking_time)     # braking ramp rate
driver.Initialize()                                         # finalize the driver

render_every = max(1, round(render_step_size / step_size))  # physics steps per rendered frame

realtime_timer = chrono.ChRealtimeStepTimer()              # wall-clock pacing
step_number = 0                                            # physics step counter
while vis.Run():
    time = system.GetChTime()                             # current sim time

    if step_number % render_every == 0:                   # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                    # current driver command

    driver.Synchronize(time)                              # update driver
    terrain.Synchronize(time)                             # update terrain
    hmmwv.Synchronize(time, driver_inputs, terrain)       # update vehicle with terrain
    vis.Synchronize(time, driver_inputs)                  # update visualization

    driver.Advance(step_size)                             # advance driver
    terrain.Advance(step_size)                            # advance terrain
    hmmwv.Advance(step_size)                              # advance vehicle (steps the owned system)
    vis.Advance(step_size)                                # advance visualization

    step_number += 1                                      # next step
    realtime_timer.Spin(step_size)                        # spin so wall-clock matches sim time
