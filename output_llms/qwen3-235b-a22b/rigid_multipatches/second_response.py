import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set the data path for PyChrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Define simulation step sizes
step_size = 2e-3
tire_step_size = 1e-3

def main():
    # Create the HMMWV vehicle with updated initial position
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

    # Set visualization types for vehicle components
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Set the collision system type
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create terrain with a single patch using a collision mesh
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Define contact material for the terrain patch
    patch_material = chrono.ChContactMaterialNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)

    # Add the collision mesh patch
    patch = terrain.AddPatch(patch_material,
                             chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                             veh.GetDataFile("terrain/meshes/Highway_col.obj"))

    # Attempt to set contact material thickness if method exists
    if hasattr(patch, 'SetContactMaterialThickness'):
        patch.SetContactMaterialThickness(0.01)

    # Load and add the visual mesh for the terrain
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), False, True)

    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(vis_mesh)
    vis_shape.SetName("Highway Visualization")

    # Attach the visual mesh to the terrain's ground body
    terrain.GetGroundBody().AddVisualShape(vis_shape, chrono.ChFramed())

    terrain.Initialize()

    # Create the Irrlicht visualization system
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

    hmmwv.GetVehicle().EnableRealtime(True)

    # Simulation loop
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Render the scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Synchronize modules
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

# Run the simulation
if __name__ == "__main__":
    main()