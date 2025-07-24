import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m




def main():
    

    

    
    hmmwv = veh.HMMWV_Full()
    hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
    hmmwv.SetChassisFixed(False);
    hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(-10, -2, 0.6), chrono.ChQuaterniond(1, 0, 0, 0)))
    hmmwv.SetEngineType(veh.EngineModelType_SIMPLE)
    hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SIMPLE_MAP)
    hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    hmmwv.SetTireType(veh.TireModelType_RIGID)
    hmmwv.Initialize()

    hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    hmmwv.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    

    terrain = veh.RigidTerrain(hmmwv.GetSystem())
    patch1 = veh.RigidTerrain.Patch()
    patch1.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch1.SetFriction(0.9)
    patch1.SetRestitution(0.01)
    patch1.SetNormal(chrono.ChVector3d(0, 0, 1))
    patch1.SetPoint(chrono.ChVector3d(0, 0, 0))
    patch1.SetSize(100.0, 100.0)
    terrain.AddPatch(patch1)

    patch2 = veh.RigidTerrain.Patch()
    patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch2.SetFriction(0.9)
    patch2.SetRestitution(0.01)
    patch2.SetNormal(chrono.ChVector3d(0, 0, 1))
    patch2.SetPoint(chrono.ChVector3d(0, -200, 0))
    patch2.SetSize(100.0, 100.0)
    terrain.AddPatch(patch2)

    patch3 = veh.RigidTerrain.Patch()
    patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)
    patch3.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch3.SetFriction(0.9)
    patch3.SetRestitution(0.01)
    patch3.SetNormal(chrono.ChVector3d(0, 0, 1))
    patch3.SetPoint(chrono.ChVector3d(200, 0, 0))
    patch3.SetSize(100.0, 100.0)
    patch3.SetMesh(veh.GetDataFile("terrain/meshes/bump.obj"))
    terrain.AddPatch(patch3)

    patch4 = veh.RigidTerrain.Patch()
    patch4.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 200, 200)
    patch4.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch4.SetFriction(0.9)
    patch4.SetRestitution(0.01)
    patch4.SetNormal(chrono.ChVector3d(0, 0, 1))
    patch4.SetPoint(chrono.ChVector3d(200, -200, 0))
    patch4.SetSize(100.0, 100.0)
    patch4.SetHeight(veh.GetDataFile("terrain/height_maps/bump64.bmp"), 0.0, 0.1)
    terrain.AddPatch(patch4)

    patch5 = veh.RigidTerrain.Patch()
    patch5.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch5.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    patch5.SetFriction(0.9)
    patch5.SetRestitution(0.01)
    patch5.SetNormal(chrono.ChVector3d(0, 0, 1))
    patch5.SetPoint(chrono.ChVector3d(-200, 0, 0))
    patch5.SetSize(100.0, 100.0)
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
    driver.SetSteeringDelta(0.06)
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

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0








veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')


step_size = 2e-3

main()