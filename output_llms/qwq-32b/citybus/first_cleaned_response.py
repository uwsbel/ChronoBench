import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurface()
    patch = veh.RigidTerrainPatch(terrain, patch_mat)
    patch.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
    terrain.Initialize()

    
    vehicle = veh.CityBus()
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))  
    vehicle.SetInitFwdVector(chrono.ChVectorD(1, 0, 0))  
    vehicle.SetTireType(veh.TireModelType.TMEASY)  
    vehicle.Initialize()
    
    
    driver = veh.ManualDriver()
    vehicle.SetDriver(driver)

    
    app = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(1280, 720))
    app.AddAsset(terrain.GetAsset())  

    
    for asset in vehicle.GetVisualizationAssets():
        app.AddAsset(asset)
    vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType.PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)

    
    cam = irr.ChIrrCameraDriver(app.GetVideoDriver())
    cam.SetChasingMode(True)
    cam.SetChaseTarget(vehicle.GetChassis())
    cam.SetChaseOffset(chrono.ChVectorD(0, 0, 2))  
    app.SetCameraDriver(cam)
    app.SetCameraPosition(chrono.ChVectorD(0, 5, 10))  

    
    app.SetTimestep(0.02)  

    
    app.MainLoop()

if __name__ == '__main__':
    main()