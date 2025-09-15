import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # ===== 1. Initialize Environment =====
    chrono.SetChronoDataPath("path/to/chrono/data/")  # Set your Chrono data path
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    # Create systems
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    # ===== 2. Create Terrain =====
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain.Initialize(patch_mat)

    # ===== 3. Create and Initialize Vehicle =====
    vehicle = veh.Kraz(system)
    vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 1.0, 0), chrono.QUNIT))
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # ===== 4. Create Driver System =====
    driver = veh.ChIrrGuiDriver(irr_env)  # Will be fully initialized after creating Irrlicht
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # ===== 5. Setup Visualization =====
    irr_env = irr.ChIrrApp(system, "Kraz Vehicle Simulation", irr.dimension2du(1280, 720))
    irr_env.AddLogo()
    irr_env.AddSkyBox()
    irr_env.AddCamera(chrono.ChVectorD(8, 2, 0), chrono.ChVectorD(0, 1, 0))
    irr_env.AddLightWithShadow(chrono.ChVectorD(-5, 8, 0), chrono.ChVectorD(0, 0, 0), 50, 1, 30, 60, 512)
    irr_env.EnableShadows()
    
    # Attach vehicle-specific visualization
    vehicle.SetIrrApp(irr_env)
    driver.SetIrrApp(irr_env)  # Connect driver to visualization

    # ===== 6. Simulation Loop =====
    step_size = 0.02
    realtime_timer = chrono.ChRealtimeStepTimer()
    irr_env.AssetBindAll()
    irr_env.AssetUpdateAll()

    while irr_env.GetDevice().run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        irr_env.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        irr_env.DrawAll()
        irr_env.EndScene()
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()