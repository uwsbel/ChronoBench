import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Initialize the Chrono system
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create rigid terrain with texture
    terrain = veh.RigidTerrain(my_system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(chrono.ChCoordsysD(),  # Local coordinates
                            100, 100,  # Size X, Z
                            patch_mat)
    patch_mat.SetFriction(0.9)
    terrain.Initialize()
    
    # Set terrain texture (example texture)
    texture_path = chrono.GetChronoDataFile('textures/concrete.jpg')
    patch.GetMaterialSurface().GetVisualMaterial().SetTexture(texture_path, 200, 200)

    # Create BMW E90 Sedan vehicle with TMEASY tires
    vehicle = veh.Sedan(my_system)
    vehicle.SetTireModelType(veh.TireModelType.TMEASY)
    vehicle.SetTireStepSize(0.01)  # Tire integration time step
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0.5, 0))  # Start above terrain
    vehicle.SetInitFwdVel(10)  # Initial forward velocity (m/s)
    
    # Configure visualization (mesh-based for better detail)
    vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType.MESH)
    
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.QUNIT),
                      veh.ChContactMethod.NSC,
                      True)  # Enable visualization

    # Initialize driver with Irrlicht input
    driver = veh.ChIrrGuiDriver()
    vehicle.SetDriver(driver)

    # Setup Irrlicht visualization
    app = irr.ChIrrApp(my_system, 'BMW E90 Sedan Simulation', irr.dimension2du(1024, 768))
    app.SetCamera( irr.ChIrrChaseCamera(app.GetSceneManager(), vehicle.GetChassis()) )
    app.GetCamera().SetChaseDirection( irr.VECT_Y, True )  # Chase along Y-axis (vehicle's forward)
    app.GetCamera().SetChasePosition( chrono.ChVectorD(0, 2, 5) )  # Position relative to chassis
    
    # Add lighting and skybox
    app.AddTypicalLights()
    app.AddLightWithShadow(chrono.ChVectorD(0, 30, 0), chrono.ChVectorD(0, 0, 0), 15, 100, 512)
    app.AddSkyBox()

    # Bind and update assets
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Simulation loop
    while app.GetDevice().run():
        time = my_system.GetChTime()
        
        # Update driver inputs (steering, throttle, brake)
        driver.Synchronize(time)
        
        # Update vehicle and terrain
        vehicle.Synchronize(time)
        terrain.Synchronize(time)
        
        # Advance simulation step
        my_system.DoStepDynamics(0.01)  # Time step matches tire step
        
        # Render scene
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

if __name__ == '__main__':
    main()