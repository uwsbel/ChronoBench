import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

def main():
    # 1. Initialize environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(veh.GetDataPath() + "vehicle/")

    # 2. Create systems
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)

    # 3. Create custom mesh terrain
    terrain = veh.RigidTerrain(system)
    mesh_patch = terrain.AddPatch(
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
        veh.GetDataFile("terrain/Highway_col.obj"),
        veh.GetDataFile("terrain/Highway_vis.obj"),
        0.01,  # contact mesh sweep sphere radius
        True   # enable visualization
    )
    mesh_patch.SetColor(chrono.ChColor(0.5, 0.6, 0.5))
    mesh_patch.SetTexture(veh.GetDataFile("terrain/texture_tile.jpg"), 20, 20)
    terrain.Initialize()

    # 4. Create HMMWV vehicle
    init_pos = chrono.ChVectorD(0, 0.5, 0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    
    hmmwv = veh.HMMWV_Full(system)
    hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    hmmwv.SetTireType(veh.TireModelType_TMEASY)
    hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
    hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)
    hmmwv.Initialize()

    # 5. Set tire parameters
    tire_lf = hmmwv.GetTire(veh.LEFT_FRONT)
    tire_rf = hmmwv.GetTire(veh.RIGHT_FRONT)
    tire_lr = hmmwv.GetTire(veh.LEFT_REAR)
    tire_rr = hmmwv.GetTire(veh.RIGHT_REAR)
    for tire in [tire_lf, tire_rf, tire_lr, tire_rr]:
        tire.SetTireMaximumLoad(8000)
        tire.Initialize()

    # 6. Create interactive driver
    driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 7. Initialize Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle('HMMWV on Mesh Terrain')
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 4, -8), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    # 8. Simulation loop
    step_size = 0.002
    frame_interval = 0.02  # 50 FPS (1/50 = 0.02)
    time = 0
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()
        
        # Update vehicle and driver
        driver.Synchronize(time)
        hmmwv.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        # Advance dynamics
        system.DoStepDynamics(step_size)
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChFrameD(), 2)  # Reference frame
        vis.EndScene()
        
        # Maintain 50 FPS rendering
        frame_number += 1
        target_time = frame_number * frame_interval
        while time < target_time:
            time = system.GetChTime()

if __name__ == "__main__":
    main()