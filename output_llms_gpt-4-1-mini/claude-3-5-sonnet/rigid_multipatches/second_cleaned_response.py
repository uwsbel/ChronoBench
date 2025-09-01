import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False)
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVectorD(6, -70, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))
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

    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)

    terrain_mesh = terrain.AddMesh(terrain_mat, veh.GetDataFile("vehicle/terrain/meshes/Highway_col.obj"), "", 0.01, False)
    terrain_mesh.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    
    terrain_body = terrain.GetGroundBody()
    vis_mesh = chrono.ChTriangleMeshConnected()
    vis_mesh.LoadWavefrontMesh(veh.GetDataFile("terrain/meshes/Highway_vis.obj"), False, False)
    
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(vis_mesh)
    vis_shape.SetName("terrain_visual")
    terrain_body.AddVisualShape(vis_shape)

    terrain.Initialize()

    
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Rigid Terrain Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 0.75), 6.0, 0.5)
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

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 2e-3
tire_step_size = 1e-3

main()