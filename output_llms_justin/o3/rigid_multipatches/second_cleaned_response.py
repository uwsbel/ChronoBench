import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr          




veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')   
STEP_SIZE       = 2e-3                                     
TIRE_STEP_SIZE  = 1e-3                                     



def main():
    
    
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)

    init_loc = chrono.ChVectorD( 6, -70, 0.5 )             
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(init_loc, init_rot))

    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)

    
    try:
        hmmwv.SetDriveType(veh.DrivelineTypeWV.AWD)
    except AttributeError:
        hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)

    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(TIRE_STEP_SIZE)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType     (veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType  (veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType    (veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType       (veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType        (veh.VisualizationType_MESH)

    sys = hmmwv.GetSystem()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    
    
    terrain = veh.RigidTerrain(sys)

    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)

    coll_mesh = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    patch_cs  = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)

    
    patch = terrain.AddPatch(patch_mat, patch_cs, coll_mesh, 0.01)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    patch.SetTexture(veh.GetDataFile("terrain/textures/road.jpg"), 1.0, 1.0)

    
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"),
                               False,  
                               True)   

    vis_shape = chrono.ChVisualShapeTriangleMesh(vis_mesh)
    vis_shape.SetName("Highway_vis")
    vis_shape.SetBackfaceCull(True)
    vis_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.4))

    
    ground_body = patch.GetGroundBody() if hasattr(patch, "GetGroundBody") \
                                        else patch.GetGround()
    ground_body.AddVisualShape(vis_shape)

    terrain.Initialize()

    
    
    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV – Highway mesh demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(hmmwv.GetVehicle())

    
    
    
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta (0.02)
    driver.SetThrottleDelta (0.02)
    driver.SetBrakingDelta  (0.06)
    driver.Initialize()

    hmmwv.GetVehicle().EnableRealtime(True)

    
    
    
    while vis.Run():
        time = sys.GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize (time)
        terrain.Synchronize(time)
        hmmwv.Synchronize  (time, driver_inputs, terrain)
        vis.Synchronize    (time, driver_inputs)

        
        driver.Advance (STEP_SIZE)
        terrain.Advance(STEP_SIZE)
        hmmwv.Advance  (STEP_SIZE)
        vis.Advance    (STEP_SIZE)

    return 0



if __name__ == '__main__':
    main()