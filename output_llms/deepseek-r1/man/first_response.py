import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # Set path to Chrono data directory
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # Initialize simulation system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.GetChronoDataFile('textures/terrain/Concrete.jpg'), 200, 200)
    patch.SetTexture(veh.GetDataFile('terrain/textures/concrete.jpg'), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create MAN truck vehicle system (using 5t model scaled to 10t)
    init_pos = chrono.ChVectorD(0, 0, 0.5)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    truck = veh.MAN_5t()
    truck.SetContactMethod(chrono.ChContactMethod_SMC)
    truck.SetChassisFixed(False)
    truck.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    truck.SetTireType(veh.TireModelType_TMEASY)
    truck.Initialize()

    # Scale vehicle to 10t mass
    chassis = truck.GetChassisBody()
    mass_scale = 10000 / chassis.GetMass()  # Target 10,000 kg
    chassis.SetMass(10000)
    chassis.SetInertiaXX(chassis.GetInertiaXX() * mass_scale)

    # Set visualization modes
    truck.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    truck.SetWheelVisualizationType(veh.VisualizationType_MESH)
    truck.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create Irrlicht visualization
    app = veh.ChIrrApp(truck.GetVehicle(), "MAN 10t Truck Simulation", irr.dimension2du(1280, 720))
    app.AddTypicalLights(
        irr.vector3df(-50, -50, 80),  # left light position
        irr.vector3df(50, -50, 80),   # right light position
        250, 150)                     # light distance and intensity
    app.AddSkyBox()
    app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 12.0, 0.5)
    app.SetTimestep(0.005)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create interactive driver
    driver = veh.ChIrrGuiDriver(app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)

    # Add custom logo on terrain
    logo = chrono.ChBodyEasyBox(3, 3, 0.1, 1000, True, True)
    logo.SetPos(chrono.ChVectorD(15, 0, 0.05))
    logo.SetBodyFixed(True)
    system.Add(logo)
    
    logo_texture = chrono.ChTexture()
    logo_texture.SetTextureFilename(chrono.GetChronoDataFile('textures/logo_chronoengine.png'))
    logo.AddAsset(logo_texture)

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()
    while app.GetDevice().run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        # Update vehicle systems
        truck.Synchronize(time, driver_inputs, terrain)
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs)
        
        # Advance simulation
        app.Advance(driver_inputs.steering)
        truck.Advance(0.01)
        system.DoStepDynamics(0.01)
        
        # Update visualization
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        realtime_timer.Spin(0.01)

if __name__ == "__main__":
    main()