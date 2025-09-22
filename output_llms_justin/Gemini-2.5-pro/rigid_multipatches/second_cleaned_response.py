import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os 

def main():
    
    step_size = 2e-3
    
    
    tire_step_size = 1e-3 

    
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

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    
    

    
    
    patch_material = chrono.ChContactMaterialNSC()
    patch_material.SetFriction(0.9)
    patch_material.SetRestitution(0.01)
    
    
    
    
    ground_body = terrain.GetGroundBody()
    ground_body.GetCollisionModel().SetDefaultSuggestedMargin(0.01)

    
    
    
    collision_mesh_file = veh.GetDataFile("terrain/meshes/Highway_col.obj")
    if not os.path.exists(collision_mesh_file):
         print(f"FATAL ERROR: Collision mesh file not found: {collision_mesh_file}")
         print(f"Ensure CHRONO_DATA_DIR is set correctly and the file exists at the expected location: {os.path.abspath(collision_mesh_file)}")
         return 1 

    
    
    terrain.AddPatch(patch_material,
                     chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
                     collision_mesh_file)
    

    
    
    
    visual_mesh_file = veh.GetDataFile("terrain/meshes/Highway_vis.obj")
    if not os.path.exists(visual_mesh_file):
        print(f"FATAL ERROR: Visual mesh file not found: {visual_mesh_file}")
        print(f"Ensure CHRONO_DATA_DIR is set correctly and the file exists at the expected location: {os.path.abspath(visual_mesh_file)}")
        return 1 

    
    
    vis_mesh_trimesh = chrono.ChTriangleMeshConnected()
    
    vis_mesh_trimesh.LoadWavefrontMesh(visual_mesh_file, False, True) 

    
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    
    vis_shape.SetMesh(vis_mesh_trimesh)
    vis_shape.SetName("highway_visual_mesh") 
    
    
    
    
    ground_body.AddVisualShape(vis_shape)
    

    terrain.Initialize() 


    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Mesh Terrain Demo') 
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

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)
        
    return 0





veh.SetDataPath(os.path.join(chrono.GetChronoDataPath(), 'vehicle', ''))


if __name__ == '__main__':
    main()