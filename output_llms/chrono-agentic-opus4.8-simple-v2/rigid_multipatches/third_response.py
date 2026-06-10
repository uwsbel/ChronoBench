import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 2e-3                                                       # dynamics step
tire_step_size = 1e-3                                                  # tire substep
sim_end = 20.0                                                         # simulation duration (s)

# Create the HMMWV vehicle, set parameters, and initialize
hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog model
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisFixed(False)                                         # chassis must be free to move
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))  # spawn on patch 1
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)                      # simple engine map
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # automatic transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                         # all-wheel drive
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                              # tire integration step
hmmwv.Initialize()

hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)        # hide chassis box
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # primitive suspension
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # primitive steering
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # mesh tires

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

# Create the terrain with multiple patches
terrain = veh.RigidTerrain(hmmwv.GetSystem())                       # rigid terrain owner

patch1_mat = chrono.ChContactMaterialNSC()                          # patch 1 contact material
patch1_mat.SetFriction(0.9)                                        # friction coefficient
patch1_mat.SetRestitution(0.01)                                   # restitution
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysd(chrono.ChVector3d(-20, 5, 0), chrono.QUNIT), 32, 20)  # flat box patch
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                    # sandy color
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tile texture

patch2_mat = chrono.ChContactMaterialNSC()                          # patch 2 contact material
patch2_mat.SetFriction(0.9)                                        # friction coefficient
patch2_mat.SetRestitution(0.01)                                   # restitution
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysd(chrono.ChVector3d(20, -5, 0.2), chrono.QUNIT), 32, 30)  # raised flat box patch
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))                    # reddish color
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # concrete texture

patch3_mat = chrono.ChContactMaterialNSC()                          # patch 3 contact material
patch3_mat.SetFriction(0.9)                                        # friction coefficient
patch3_mat.SetRestitution(0.01)                                   # restitution
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysd(chrono.ChVector3d(5, -45, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # triangle-mesh bump patch
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                    # bluish color
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture

patch4_mat = chrono.ChContactMaterialNSC()                          # patch 4 contact material
patch4_mat.SetFriction(0.9)                                        # friction coefficient
patch4_mat.SetRestitution(0.01)                                   # restitution
patch4 = terrain.AddPatch(patch4_mat, chrono.ChCoordsysd(chrono.ChVector3d(10, 40, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/height_maps/bump64.bmp"), 64.0, 64.0, 0.0, 3.0)  # heightmap patch
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 6.0, 6.0)  # grass texture

terrain.Initialize()                                               # build terrain collision + visuals

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                   # vehicle-aware Irrlicht system
vis.SetWindowTitle('HMMWV Rigid Terrain Demo')                    # window title
vis.SetWindowSize(1280, 1024)                                     # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)  # chase camera on chassis
vis.Initialize()                                                 # build device first
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) # logo
vis.AddLightDirectional()                                        # directional light (vehicle scenes)
vis.AddSkyBox()                                                  # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                            # bind vehicle visual assets

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)                          # keyboard driver bound to vis
driver.SetSteeringDelta(0.02)                                    # steering rate
driver.SetThrottleDelta(0.02)                                    # throttle rate
driver.SetBrakingDelta(0.06)                                     # braking rate
driver.Initialize()

hmmwv.GetVehicle().EnableRealtime(True)                           # real-time pacing

render_step_size = 1.0 / 50.0                                     # 50 fps render cadence
render_every = max(1, round(render_step_size / step_size))       # untagged cadence constant

while vis.Run() and hmmwv.GetSystem().GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = hmmwv.GetSystem().GetChTime()                     # current sim time

        driver_inputs = driver.GetInputs()                       # current driver inputs

        driver.Synchronize(time)                                 # sync driver
        terrain.Synchronize(time)                                # sync terrain
        hmmwv.Synchronize(time, driver_inputs, terrain)          # sync vehicle with terrain
        vis.Synchronize(time, driver_inputs)                     # sync visualization

        driver.Advance(step_size)                                # advance driver
        terrain.Advance(step_size)                               # advance terrain
        hmmwv.Advance(step_size)                                 # advance vehicle (steps the system)
        vis.Advance(step_size)                                   # advance visualization

        if hmmwv.GetSystem().GetChTime() >= sim_end:
            break
