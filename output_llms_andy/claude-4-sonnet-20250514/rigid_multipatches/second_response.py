import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Set data path at the beginning
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Simulation step sizes
    step_size = 2e-3
    tire_step_size = 1e-3
    
    #  Create the HMMWV vehicle, set parameters, and initialize
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    # Updated initial position as per instruction 1
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(tire_step_size)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the simplified terrain with single patch using mesh
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    # Create contact material with same friction and restitution values
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    # Add single terrain patch using Highway collision mesh
    patch = terrain.AddPatch(patch_mat, 
                            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                            veh.GetDataFile("terrain/meshes/Highway_col.obj"))
    
    # Set contact material thickness
    patch.SetContactMaterialThickness(0.01)
    
    # Create and add visual mesh for the terrain
    try:
        # Load the visual mesh
        vis_mesh = chrono.ChTriangleMeshConnected()
        vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"))
        
        # Create visual shape from the mesh
        vis_shape = chrono.ChVisualShapeTriangleMesh()
        vis_shape.SetMesh(vis_mesh)
        vis_shape.SetName("Highway_visual")
        
        # Add visual shape to the terrain's ground body
        terrain.GetGroundBody().AddVisualShape(vis_shape)
        
    except Exception as e:
        print(f"Warning: Could not load visual mesh: {e}")
        # Fallback: set basic color for the patch
        patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))

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

    hmmwv.GetVehicle().EnableRealtime(True)

    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        # Draw scene
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

    return 0

if __name__ == "__main__":
    main()