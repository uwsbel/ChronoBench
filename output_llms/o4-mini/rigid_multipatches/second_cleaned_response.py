import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr





veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size      = 2e-3
tire_step_size = 1e-3

def main():
    
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    
    init_pos = chrono.ChCoordsysd(
        chrono.ChVector3d(6.0, -70.0, 0.5),
        chrono.ChQuaterniond(1, 0, 0, 0))
    hmmwv.SetInitPosition(init_pos)
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

    
    
    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    
    mesh_file_col = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    patch = terrain.AddPatch(
        patch_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
        mesh_file_col)
    
    patch.SetContactMaterialThickness(0.01)

    
    mesh_file_vis = veh.GetDataFile("terrain/meshes/Highway_vis.obj")
    tri_mesh = chrono.ChTriangleMeshConnected()
    
    tri_mesh.LoadWavefrontMesh(mesh_file_vis, True, False)

    vis_shape = chrono.ChVisualShapeTriangleMesh(
        tri_mesh,
        True,   
        False,  
        chrono.ChColor(1, 1, 1),
        mesh_file_vis)
    patch.GetGroundBody().AddVisualShape(vis_shape)

    terrain.Initialize()

    
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Single‐Mesh Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    hmmwv.GetVehicle().EnableRealtime(True)

    
    
    
    while vis.Run():
        time = hmmwv.GetSystem().GetChTime()

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        inputs = driver.GetInputs()
        hmmwv.Synchronize(time, inputs, terrain)
        vis.Synchronize(time, inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0

if __name__ == "__main__":
    main()