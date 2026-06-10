import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

init_loc = chrono.ChVector3d(-10, -2, 0.6)                           # HMMWV spawn over the first flat patch
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # QUNIT — no yaw

step_size = 2e-3                                                      # integration step
tire_step_size = 1e-3                                                 # tire model substep
render_step_size = 1.0 / 50.0                                         # 50 FPS render cadence

hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
hmmwv.SetChassisFixed(False)                                         # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)                      # prompt: engine type
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # paired transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                          # prompt: drivetrain type — all-wheel drive
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

terrain = veh.RigidTerrain(system)                                   # multi-patch rigid terrain

patch1_mat = chrono.ChContactMaterialNSC()                           # patch 1 material
patch1_mat.SetFriction(0.9)
patch1_mat.SetRestitution(0.01)
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT), 32, 20)  # flat box patch
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)

patch2_mat = chrono.ChContactMaterialNSC()                           # patch 2 material
patch2_mat.SetFriction(0.9)
patch2_mat.SetRestitution(0.01)
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT), 32, 30)  # second flat box, slightly raised
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)

patch3_mat = chrono.ChContactMaterialNSC()                           # patch 3 material
patch3_mat.SetFriction(0.9)
patch3_mat.SetRestitution(0.01)
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # mesh-based bump patch
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6, 6)

patch4_mat = chrono.ChContactMaterialNSC()                           # patch 4 material
patch4_mat.SetFriction(0.9)
patch4_mat.SetRestitution(0.01)
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)  # heightmap patch
patch4.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6, 6)

terrain.Initialize()                                                 # finalize all patches

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
