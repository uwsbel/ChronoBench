import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(6, -70, 0.5), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetTireStepSize(1e-3)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    terrain = veh.RigidTerrain(hmmwv.GetSystem())

    
    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)

    
    terrain_mesh = chrono.ChTriangleMesh()
    terrain_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_col.obj"))
    terrain_mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain_vis_mesh = chrono.ChTriangleMesh()
    terrain_vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"))
    terrain_vis_mesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))

    
    terrain.AddMesh(terrain_mesh, terrain_mat, 0.01)

    
    terrain_vis_shape = chrono.ChVisualShapeTriangleMesh(terrain_vis_mesh)
    terrain_vis_shape.SetColor(chrono.ChColor(0.5, 0.5, 0.8))
    terrain.GetGroundBody().AddVisualShape(terrain_vis_shape)

    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
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

    while vis.Run() :
        time = hmmwv.GetSystem().GetChTime()

        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(time)
        terrain.Synchronize(time)
        hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize(time, driver_inputs)

        
        driver.Advance(2e-3)
        terrain.Advance(2e-3)
        hmmwv.Advance(2e-3)
        vis.Advance(2e-3)

    return 0

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

main()