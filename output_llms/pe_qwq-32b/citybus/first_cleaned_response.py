import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('CityBus Simulation')
    vis.AttachSystem(sys)
    vis.SetCameraPosition(chrono.ChVectorD(0, 5, 30))
    vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2 / 2, chrono.VECT_Y))
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    
    citybus = veh.CityBus()
    citybus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    citybus.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    citybus.SetTireType(veh.TireModelType.RIGID)  
    citybus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    citybus.Initialize()

    
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 200)  
    terrain_texture = chrono.GetChronoDataFile('textures/concrete.jpg')
    patch = terrain.GetPatch(0)
    texture_asset = irr.ChTexture()
    texture_asset.SetTextureFilename(terrain_texture)
    patch.AddAsset(texture_asset)

    
    driver = irr.ChIrrGuiDriver(vis)
    citybus.SetDriver(driver)

    
    time_step = 0.02  
    time = 0

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        driver.Synchronize(time_step)

        
        sys.DoStepDynamics(time_step)
        time += time_step

        
        chassis = citybus.GetChassis()
        bus_pos = chassis.GetPos()
        cam_pos = bus_pos + chrono.ChVectorD(0, 5, -10)
        vis.GetCamera().SetPos(cam_pos)
        vis.GetCamera().SetLookAt(bus_pos)

if __name__ == '__main__':
    main()