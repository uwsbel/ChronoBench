import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Initialize Chrono system
    system = chrono.ChSystem()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Create terrain
    terrain = veh.RigidTerrain(system, chrono.ChMaterialSurfaceNSC())
    terrain.SetHeight(0)
    terrain.SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'), 200, 200)
    terrain.Initialize()

    # Create BMW E90 Sedan vehicle with TMEASY tires
    my_vehicle = veh.Sedan(system)
    my_vehicle.SetTireModelType(veh.TireModelType.TMEASY)
    my_vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
    my_vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
    my_vehicle.SetInitPosition(chrono.ChVectorD(0, 0.2, 0.5), chrono.ChQuaternionD(1,0,0,0))
    my_vehicle.Initialize()

    # Configure TMEasy tires (optional parameters)
    for axle in my_vehicle.GetAxles():
        for wheel in axle.wheels:
            wheel.tire.SetTireType(veh.TireModelType.TMEASY)
            # Example parameter tuning (uncomment as needed)
            # wheel.tire.GetTMEasy().SetLongitudinalStiffness(1e5)
            # wheel.tire.GetTMEasy().SetLateralStiffness(1e5)

    # Create interactive driver
    driver = veh.ManualDriver()
    my_vehicle.InitializeDriver(driver)

    # Initialize Irrlicht visualization
    app = irr.ChIrrApp(system, 'BMW E90 Sedan Simulation', irr.dimension2du(1280, 720))
    app.AddLightWithShadow(chrono.ChVectorD(0, 30, 50), chrono.ChVectorD(0, 0, 0), 250, 500, 250)
    app.SetSkyBox()
    app.SetChaseCamera(my_vehicle.GetChassis(), 6.0, 0.5)  # Chase camera setup
    app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.Add(terrain.GetPlane())  # Add terrain visualization
    app.Add(my_vehicle.GetVisualization())  # Add vehicle visualization

    # Simulation settings
    app.SetTimestep(0.01)
    app.SetTryRealtime(True)

    # Start simulation loop
    app.MainLoop()

if __name__ == '__main__':
    main()