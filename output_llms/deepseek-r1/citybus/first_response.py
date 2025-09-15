import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize PyChrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    # 2. Create and initialize CityBus vehicle
    init_pos = chrono.ChVectorD(0, 0, 1.0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    bus = veh.CityBus()
    bus.SetContactMethod(chrono.ChContactMethod_NSC)
    bus.SetChassisCollisionType(veh.CollisionType_NONE)
    bus.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    bus.SetTireType(veh.TireModelType_TMEASY)
    bus.Initialize()

    # Set visualization types
    bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
    bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    bus.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 3. Create RigidTerrain with custom texture
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                            chrono.GetChronoDataFile("vehicle/terrain/meshes/terrain.obj"), 
                            "terrain_textures/concrete.jpg", 0.01)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    terrain.Initialize()

    # 4. Initialize Irrlicht visualization
    app = veh.ChVehicleIrrApp(bus.GetVehicle(), "CityBus Simulation", 800, 600)
    app.AddTypicalLights()
    app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(chrono.ChVectorD(0.0, -8.0, 2.0), 8.0, 0.5)
    app.SetTimestep(0.02)
    app.AssetBindAll()
    app.AssetUpdateAll()

    # 5. Create interactive driver system
    driver = veh.ChInteractiveDriverIRR(app)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # 6. Simulation loop (50 FPS)
    step_size = 0.02
    realtime_timer = chrono.ChRealtimeStepTimer()
    while app.GetDevice().run():
        time = system.GetChTime()
        
        # Render scene
        app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()
        
        # Get driver inputs
        driver_inputs = driver.GetInputs()
        
        # Update systems
        bus.Synchronize(time, driver_inputs)
        terrain.Synchronize(time)
        app.Synchronize("", driver_inputs)
        
        # Advance simulation
        app.Advance(step_size)
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()