import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize simulation system
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # 2. Create visualization system
    vis = irr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('CityBus Simulation')
    vis.AttachSystem(sys)
    vis.SetCameraPosition(chrono.ChVectorD(0, 5, 30))
    vis.SetCameraRotation(chrono.Q_from_AngAxis(chrono.CH_C_PI_2 / 2, chrono.VECT_Y))
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    # 3. Create CityBus vehicle
    citybus = veh.CityBus()
    citybus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    citybus.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    citybus.SetTireType(veh.TireModelType.RIGID)  # Simple rigid tire model
    citybus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    citybus.Initialize()

    # 4. Create rigid terrain with custom texture
    terrain = veh.RigidTerrain(sys)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 200)  # 200m x 200m terrain
    terrain_texture = chrono.GetChronoDataFile('textures/concrete.jpg')
    patch = terrain.GetPatch(0)
    texture_asset = irr.ChTexture()
    texture_asset.SetTextureFilename(terrain_texture)
    patch.AddAsset(texture_asset)

    # 5. Setup interactive driver controls
    driver = irr.ChIrrGuiDriver(vis)
    citybus.SetDriver(driver)

    # 6. Simulation loop parameters
    time_step = 0.02  # 50Hz simulation rate
    time = 0

    # 7. Main simulation loop
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Update driver controls
        driver.Synchronize(time_step)

        # Advance simulation
        sys.DoStepDynamics(time_step)
        time += time_step

        # Update camera to follow vehicle
        chassis = citybus.GetChassis()
        bus_pos = chassis.GetPos()
        cam_pos = bus_pos + chrono.ChVectorD(0, 5, -10)
        vis.GetCamera().SetPos(cam_pos)
        vis.GetCamera().SetLookAt(bus_pos)

if __name__ == '__main__':
    main()