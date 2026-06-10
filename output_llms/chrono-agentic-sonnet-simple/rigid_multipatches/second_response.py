import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

# Simulation step sizes
step_size = 2e-3                                                     # physics time step (s)
tire_step_size = 1e-3                                                # tire sub-step size (s)
render_fps = 50                                                      # target render rate
render_every = max(1, round(1.0 / (render_fps * step_size)))        # cadence (untagged)
step_number = 0                                                      # step counter (untagged)

# Create the HMMWV vehicle, set parameters, and initialize
hmmwv = veh.HMMWV_Full()                                            # full model wrapper
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
hmmwv.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
hmmwv.SetChassisFixed(False)                                        # MANDATORY — fixed won't move
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))  # initial position
hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)                     # simple engine model
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)  # auto transmission
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)                        # all-wheel drive
hmmwv.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tire model
hmmwv.SetTireStepSize(tire_step_size)                               # tire sub-step
hmmwv.Initialize()                                                   # initialize vehicle

hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)       # mesh chassis visual
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)    # mesh suspension visual
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)      # mesh steering visual
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)         # mesh wheel visual
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)          # mesh tire visual

hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED, after Initialize

print("VEHICLE MASS: ", hmmwv.GetVehicle().GetMass())               # report total vehicle mass

# Create the terrain — single mesh patch (Highway_col.obj) with visual mesh
patch_mat = chrono.ChContactMaterialNSC()                           # NSC contact material
patch_mat.SetFriction(0.9)                                          # friction coefficient
patch_mat.SetRestitution(0.01)                                      # restitution

terrain = veh.RigidTerrain(hmmwv.GetSystem())                       # rigid terrain
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),  # patch at origin
    chrono.GetChronoDataFile('vehicle/terrain/meshes/Highway_col.obj'),  # collision mesh
    True, 0.01, False)                                              # collision, thickness, wireframe

# Add visual mesh for terrain (Highway_vis.obj)
vis_mesh = chrono.ChTriangleMeshConnected().CreateFromWavefrontFile(
    veh.GetDataFile("terrain/meshes/Highway_vis.obj"), True, True)  # visual mesh
tri_mesh_shape = chrono.ChVisualShapeTriangleMesh()                 # visual shape wrapper
tri_mesh_shape.SetMesh(vis_mesh)                                    # assign mesh
tri_mesh_shape.SetMutable(False)                                    # static mesh
patch.GetGroundBody().AddVisualShape(tri_mesh_shape)                # attach to ground body

terrain.Initialize()                                                 # initialize terrain

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle visual system
vis.SetWindowTitle('HMMWV Rigid Terrain Demo')                      # window title
vis.SetWindowSize(1280, 1024)                                       # window size
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)   # chase camera
vis.Initialize()                                                     # FIRST: initialize vis
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))    # logo
vis.AddLightDirectional()                                            # directional light
vis.AddSkyBox()                                                      # sky box
vis.AttachVehicle(hmmwv.GetVehicle())                               # attach vehicle

# Create the interactive driver system
driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver
driver.SetSteeringDelta(0.02)                                       # steering response rate
driver.SetThrottleDelta(0.02)                                       # throttle response rate
driver.SetBrakingDelta(0.06)                                        # braking response rate
driver.Initialize()                                                  # initialize driver

hmmwv.GetVehicle().EnableRealtime(True)                             # real-time stepping


while vis.Run():
    time = hmmwv.GetSystem().GetChTime()                            # current simulation time

    if step_number % render_every == 0:                             # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()


    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Synchronize all subsystems
    driver.Synchronize(time)
    terrain.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all subsystems
    driver.Advance(step_size)
    terrain.Advance(step_size)
    hmmwv.Advance(step_size)
    vis.Advance(step_size)

    step_number += 1                                                # advance step counter
