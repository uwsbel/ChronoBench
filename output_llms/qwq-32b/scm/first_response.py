import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import sys

def main():
    # Create the Chrono system
    my_system = chrono.ChSystemNSC()

    # Create the Irrlicht application
    app = irr.ChIrrApp(my_system, 'HMMWV on SCM Terrain', irr.dimension2du(1024, 768))
    app.SetSymbolscale(0.01)
    app.AddTypicalLogo()
    app.AddTypicalSky()
    app.AddTypicalLights()
    app.AddTypicalCamera(chrono.ChVectorD(0, 0, 3), chrono.ChVectorD(0, 0, 0))

    # Create the HMMWV vehicle
    vehicle = veh.HMMWV()
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    vehicle.SetTireType(veh.TireModelType.RIGID)
    vehicle.SetVisualizationType(veh.VisualizationType.MESH)
    
    # Initialize vehicle system (required for interactions)
    vehicle_system = veh.ChWheeledVehicleSystem()
    vehicle_system.Initialize(my_system, vehicle, None, False)  # Temporarily without terrain

    # Create SCM soil model
    soil = veh.SoilModelSCM()
    soil.SetCohesion(0.1)
    soil.SetFrictionAngle(chrono.CH_C_PI_4)  # 45 degrees
    soil.SetRelativeDensity(0.7)
    soil.SetThickness(0.5)
    soil.SetMoisture(0.0)

    # Create deformable terrain with moving patch
    terrain = veh.ChDeformableTerrain(my_system)
    terrain.SetSoilModel(soil)
    terrain.SetSize(20, 20)  # 20m x 20m terrain
    terrain.SetVisualizationType(veh.VisualizationType.GRID)
    terrain.SetDrawSinkage(True)  # Enable false color sinkage visualization

    # Add a moving patch following the chassis
    moving_patch = veh.ChTerrainPatch()
    moving_patch.SetSize(5, 5)  # 5m x 5m patch
    moving_patch.SetPosition(chrono.ChVectorD(0, 0, 0))  # Initial position
    terrain.AddPatch(moving_patch)

    # Re-initialize vehicle system with terrain
    vehicle_system.SetTerrain(terrain)
    vehicle_system.Initialize(my_system, vehicle, terrain, False)

    # Add visualization for vehicle and terrain
    app.Add(vehicle.GetVisualization())  # Explicitly add vehicle visualization
    app.Add(terrain.GetVisualization())

    # Create driver for user controls
    driver = veh.ChDriver()
    vehicle.SetDriver(driver)

    # Simulation settings
    app.SetTimestep(0.02)  # 50 FPS (real-time simulation)
    app.SetRealTimeFactor(1.0)

    # Main simulation loop
    while app.Run():
        app.BeginScene()
        app.DrawAll()
        app.DoStep()

        # Update driver inputs
        key = app.GetKeyboard()
        if key.keyIsPressed(irr.KEY_KEY_UP):
            driver.SetThrottle(1.0)
        elif key.keyIsPressed(irr.KEY_KEY_DOWN):
            driver.SetThrottle(-1.0)
        else:
            driver.SetThrottle(0.0)
        if key.keyIsPressed(irr.KEY_KEY_LEFT):
            driver.SetSteering(-1.0)
        elif key.keyIsPressed(irr.KEY_KEY_RIGHT):
            driver.SetSteering(1.0)
        else:
            driver.SetSteering(0.0)
        if key.keyIsPressed(irr.KEY_KEY_SPACE):
            driver.SetBrake(1.0)
        else:
            driver.SetBrake(0.0)

        # Update moving patch position based on chassis location
        chassis_pos = vehicle.GetChassis().GetPos()
        moving_patch.SetPosition(chrono.ChVectorD(chassis_pos.x, chassis_pos.y, 0))

        app.EndScene()

if __name__ == '__main__':
    main()