import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(6, -70, 0.5)                            # HMMWV spawn on the highway mesh
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT — no yaw

step_size = 2e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire model substep
render_step_size = 1.0 / 50.0                                         # 50 FPS render cadence

hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)                      # engine type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # paired transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                          # all-wheel drive
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire on rigid terrain
hmmwv.SetTireStepSize(tire_step_size)                                # tire substep
hmmwv.Initialize()                                                   # build the vehicle

system = hmmwv.GetSystem()                                            # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # report total vehicle mass

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)        # mesh visualization on all components
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

terrain = veh.RigidTerrain(system)                                   # rigid terrain with a single mesh patch

patch_mat = chrono.ChContactMaterialNSC()                            # patch contact material
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                         veh.GetDataFile("terrain/meshes/Highway_col.obj"), True, 0.01, False)  # collision mesh, thickness 0.01

vis_mesh = chrono.ChTriangleMeshConnected()                          # load separate visual mesh for the terrain
vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)
vis_shape = chrono.ChVisualShapeTriangleMesh()                       # visual shape wrapping the mesh
vis_shape.SetMesh(vis_mesh)
vis_shape.SetMutable(False)
patch.GetGroundBody().AddVisualShape(vis_shape)                      # attach visual mesh to the ground body

terrain.Initialize()                                                 # finalize the terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-specific Irrlicht visual system
vis.SetWindowTitle('Multipatch Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)     # chase camera behind chassis
vis.Initialize()                                                     # Irrlicht: Initialize first
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(hmmwv.GetVehicle())                                # bind the vehicle to the vis

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive steering/throttle/braking
steering_time = 1.0                                                  # s to go 0 -> +1 steering
throttle_time = 1.0                                                  # s to go 0 -> +1 throttle
braking_time = 0.3                                                   # s to go 0 -> +1 brake
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

hmmwv.GetVehicle().EnableRealtime(True)                              # real-time pacing

render_steps = math.ceil(render_step_size / step_size)               # untagged cadence constant

step_number = 0
while vis.Run():
    time = system.GetChTime()

    if step_number % render_steps == 0:                              # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()

    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)                                         # advances the wrapper-owned system
    vis.Advance(step_size)


    step_number += 1
