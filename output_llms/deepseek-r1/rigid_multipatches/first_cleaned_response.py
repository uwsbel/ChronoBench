import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import os




def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(150)

    
    terrain = veh.RigidTerrain(system)
    
    
    patch1_mat = chrono.ChMaterialSurfaceNSC()
    patch1 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 40, 20)
    patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 40, 20)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    
    
    patch2 = terrain.AddPatch(patch1_mat, chrono.ChVectorD(40, 0, 0.1), chrono.ChVectorD(0, 0, 1), 40, 40)
    patch2.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 40, 40)
    patch2.SetColor(chrono.ChColor(0.4, 0.8, 0.4))
    
    
    mesh_bump = chrono.ChTriangleMeshConnected()
    mesh_bump.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/bump.obj"))
    bump_mat = chrono.ChMaterialSurfaceNSC()
    bump_shape = chrono.ChTriangleMeshShape()
    bump_shape.SetMesh(mesh_bump)
    bump_body = chrono.ChBody()
    bump_body.AddAsset(bump_shape)
    bump_body.SetPos(chrono.ChVectorD(20, -10, 0))
    bump_body.SetBodyFixed(True)
    system.Add(bump_body)
    
    
    heightmap = veh.RigidTerrain.HF_Patch(system)
    heightmap.SetTexture(veh.GetDataFile("terrain/height_maps/height_map.png"))
    heightmap.Initialize(veh.GetDataFile("terrain/height_maps/height_map.bmp"), 
                         chrono.ChVectorD(80, -20, 0), 
                         chrono.ChVectorD(100, 0, 0), 
                         chrono.ChVectorD(0, 100, 0), 
                         0, 0, 5, 5)
    
    terrain.Initialize()

    
    hmmwv = veh.HMMWV_Reduced(system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
    hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_SHAFTS)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    
    hmmwv.Initialize()
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    
    driver = veh.ChIrrGuiDriver(irr_app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    hmmwv.GetVehicle().SetDriver(driver)

    
    irr_app = irr.ChIrrApp(system, "HMMWV on Complex Terrain", irr.dimension2du(1280, 720))
    irr_app.AddTypicalLights()
    irr_app.AddTypicalCamera(irr.vector3df(0, 10, 2), irr.vector3df(0, 0, 0))
    irr_app.AddSkyBox()
    irr_app.AssetBindAll()
    irr_app.AssetUpdateAll()

    
    step_size = 0.01
    realtime_step = True

    while irr_app.GetDevice().run():
        time = system.GetChTime()
        
        
        irr_app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        irr_app.DrawAll()
        irr_app.EndScene()
        
        
        driver.Synchronize(time)
        
        
        hmmwv.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        
        system.DoStepDynamics(step_size)
        
        
        irr_app.Synchronize(driver.GetInputModeAsString(), driver.GetInputs())
        hmmwv.Advance(step_size)
        irr_app.Advance(step_size)




if __name__ == '__main__':
    main()