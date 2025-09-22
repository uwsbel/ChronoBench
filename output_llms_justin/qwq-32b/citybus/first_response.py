import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Initialize core simulation system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create rigid terrain with custom texture
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurface()
    patch = veh.RigidTerrainPatch(terrain, patch_mat)
    patch.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
    terrain.Initialize()

    # Configure and initialize the CityBus vehicle
    vehicle = veh.CityBus()
    vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1))  # Position above terrain
    vehicle.SetInitFwdVector(chrono.ChVectorD(1, 0, 0))  # Facing along X-axis
    vehicle.SetTireType(veh.TireModelType.TMEASY)  # Specify tire model
    vehicle.Initialize()
    
    # Set up interactive driver (keyboard controls)
    driver = veh.ManualDriver()
    vehicle.SetDriver(driver)

    # Initialize Irrlicht visualization
    app = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(1280, 720))
    app.AddAsset(terrain.GetAsset())  # Add terrain visualization

    # Add vehicle visualization assets with mixed mesh/primitive types
    for asset in vehicle.GetVisualizationAssets():
        app.AddAsset(asset)
    vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType.PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)

    # Configure camera to follow the vehicle
    cam = irr.ChIrrCameraDriver(app.GetVideoDriver())
    cam.SetChasingMode(True)
    cam.SetChaseTarget(vehicle.GetChassis())
    cam.SetChaseOffset(chrono.ChVectorD(0, 0, 2))  # Position relative to vehicle
    app.SetCameraDriver(cam)
    app.SetCameraPosition(chrono.ChVectorD(0, 5, 10))  # Initial camera position

    # Set simulation step and frame rate
    app.SetTimestep(0.02)  # 50 Hz simulation step

    # Start simulation loop
    app.MainLoop()

if __name__ == '__main__':
    main()