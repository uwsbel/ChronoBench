import math                                                            # render-cadence math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # anchor the bundled vehicle subtree

step_size = 2e-3                                                       # dynamics step
tire_step_size = 1e-3                                                  # tire sub-step

#  Create the HMMWV vehicle, set parameters, and initialize
hmmwv = veh.HMMWV_Full()                                              # full HMMWV catalog wrapper (owns its system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
hmmwv.SetChassisFixed(False)                                          # MANDATORY — fixed chassis won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))  # spawn pose
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)                       # simple engine map
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # simple auto transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                          # all-wheel drive
hmmwv.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tires
hmmwv.SetTireStepSize(tire_step_size)                                # tire integration step
hmmwv.Initialize()                                                    # build the vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)        # hide chassis box
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # show suspension links
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # show steering links
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)          # mesh wheels
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)           # mesh tires

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for terrain contact
print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())                # report total vehicle mass

# Create the terrain with multiple patches
terrain = veh.RigidTerrain(hmmwv.GetSystem())                        # rigid terrain on the vehicle's system

patch1_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for patch 1
patch1_mat.SetFriction(0.9)                                          # high grip
patch1_mat.SetRestitution(0.01)                                      # almost no bounce
patch1 = terrain.AddPatch(patch1_mat, chrono.ChCoordsysd(chrono.ChVector3d(-16, 0, 0), chrono.QUNIT), 32, 20)  # flat tile patch
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # sandy tint
patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 20, 20)  # tiled texture

patch2_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for patch 2
patch2_mat.SetFriction(0.9)                                          # high grip
patch2_mat.SetRestitution(0.01)                                      # almost no bounce
patch2 = terrain.AddPatch(patch2_mat, chrono.ChCoordsysd(chrono.ChVector3d(16, 0, 0.15), chrono.QUNIT), 32, 30)  # raised concrete patch
patch2.SetColor(chrono.ChColor(1.0, 0.5, 0.5))                       # reddish tint
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 20, 20)  # concrete texture

patch3_mat = chrono.ChContactMaterialNSC()                           # NSC contact material for the bump-mesh patch
patch3_mat.SetFriction(0.9)                                          # high grip
patch3_mat.SetRestitution(0.01)                                      # almost no bounce
patch3 = terrain.AddPatch(patch3_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, -42, 0), chrono.QUNIT),
                          veh.GetDataFile("terrain/meshes/bump.obj"))  # mesh patch from bump.obj at (0, -42, 0)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.8))                       # bluish tint
patch3.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 6.0, 6.0)  # dirt texture, 6x6 scaling

terrain.Initialize()                                                  # finalize all patches

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle-aware Irrlicht window
vis.SetWindowTitle('HMMWV Rigid Terrain Demo')                       # window title
vis.SetWindowSize(1280, 1024)                                        # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)     # chase the chassis
vis.Initialize()                                                      # build device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))     # branding logo
vis.AddLightDirectional()                                            # vehicle demos use a directional light
vis.AddSkyBox()                                                       # sky backdrop
vis.AttachVehicle(hmmwv.GetVehicle())                                # bind chassis/wheel/tire visuals

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)                             # keyboard-style interactive driver
driver.SetSteeringDelta(0.02)                                        # steering rate
driver.SetThrottleDelta(0.02)                                        # throttle rate
driver.SetBrakingDelta(0.06)                                         # braking rate
driver.Initialize()                                                  # build the driver

hmmwv.GetVehicle().EnableRealtime(True)                              # pace to wall-clock

render_step_size = 1.0 / 50.0                                         # 50 fps render cadence
render_every = max(1, math.ceil(render_step_size / step_size))       # untagged cadence constant
step_number = 0                                                       # render-cadence counter

while vis.Run():
    time = hmmwv.GetSystem().GetChTime()

    # Draw scene
    if step_number % render_every == 0:                              # throttle rendering to ~50 fps
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1                                                 # advance render counter
