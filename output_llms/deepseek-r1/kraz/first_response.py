import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath() + '\\vehicle\\')
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)
    system.SetMaxPenetrationRecoverySpeed(4.0)

    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # 3. Create and initialize Kraz vehicle
    kraz = veh.Kraz(system)
    kraz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
    kraz.Initialize()
    kraz.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    kraz.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # 4. Create driver system
    driver = veh.ChIrrGuiDriver(irr_env)  # Will be set after creating Irrlicht
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # 5. Set up real-time visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Kraz Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 4, 1.5), chrono.ChVectorD(0, 0, 1))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, 5), chrono.ChVectorD(0, 0, 0.5), 50, 3, 30, 15, 512)

    # 6. Simulation loop
    step_size = 0.002
    realtime_timer = chrono.ChRealtimeStepTimer()
    driver.SetInputDataFile("")
    driver.Initialize()

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle
        kraz.Synchronize(time, driver.GetInputs(), terrain)
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        kraz.Advance(step_size)
        terrain.Advance(step_size)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        realtime_timer.Spin(step_size)

if __name__ == "__main__":
    main()