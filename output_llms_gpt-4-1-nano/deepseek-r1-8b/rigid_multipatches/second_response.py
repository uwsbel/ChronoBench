import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    # Set visualization type for vehicle parts
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Initialize collision system
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Create terrain patch with contact material
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch_mat.SetThickness(0.01)  # Set contact material thickness

    # Add single terrain patch using the highway mesh
    terrain_patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                                      veh.GetDataFile("vehicle/terrain/meshes/Highway_col.obj"), 64.0, 64.0, 0.0, 3.0)
    terrain_patch.SetColor(chrono.ChColor(1.0, 0.5, 0.5))
    terrain_patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64.0, 64.0)

    # Enable collision for the terrain's ground body
    terrain.GetBody().SetCollisionEnabled(True)

    # Create visual mesh for terrain
    visual_mesh = veh.ChVisualShapeTriangleMesh()
    visual_mesh.SetFileName(veh.GetDataFile("terrain/meshes/Highway_vis.obj"))
    visual_mesh.Create()

    # Attach visual mesh to the terrain's ground body
    terrain.GetBody().AddVisualShape(visual_mesh)

    terrain.Initialize()

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    # Create the interactive driver system
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Enable real-time simulation
    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules with current time
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3


if __name__ == "__main__":
    main()