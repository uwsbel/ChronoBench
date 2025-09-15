import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set data paths
chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
veh.SetDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/vehicle/'))

def main():
    # 1. Initialize system
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)

    # 2. Create terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch_mat.SetFriction(0.9)
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # 3. Create and initialize FEDA vehicle
    init_pos = chrono.ChVectorD(0, 0.5, 0)
    init_rot = chrono.ChQuaternionD(1, 0, 0, 0)
    feda = veh.FEDA_Vehicle(system)
    feda.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
    feda.SetTireType(veh.TireModelType_RIGID)
    feda.Initialize()

    # Set visualization modes
    feda.SetChassisVisualizationType(veh.VisualizationType_MESH)
    feda.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    feda.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    feda.SetWheelVisualizationType(veh.VisualizationType_MESH)
    feda.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 4. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("FEDA Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(0, 1, -10))
    vis.AddTypicalLights()

    # 5. Create interactive driver
    driver = veh.ChIrrGuiDriver(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    feda.GetVehicle().SetChassisDriver(driver)

    # 6. Configure vehicle systems
    feda.GetVehicle().SetChassisVisualizationType(veh.VisualizationType_MESH)
    feda.GetVehicle().SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    feda.GetVehicle().SetSteeringVisualizationType(veh.VisualizationType_MESH)
    feda.GetVehicle().SetWheelVisualizationType(veh.VisualizationType_MESH)
    feda.GetVehicle().SetTireVisualizationType(veh.VisualizationType_MESH)

    # 7. Simulation loop
    step_size = 0.002
    render_step = 1.0 / 50  # 50 FPS

    realtime_timer = chrono.ChRealtimeStepTimer()
    frame_number = 0

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle
        feda.Synchronize(time, driver.GetInputs(), terrain)
        
        # Advance dynamics
        system.DoStepDynamics(step_size)
        
        # Update visualization
        if frame_number % int(render_step / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.RenderFrame(init_pos, init_rot, 2)  # Reference frame
            vis.EndScene()
        
        # Update driver GUI
        driver.Advance(step_size)
        realtime_timer.Spin(step_size)
        frame_number += 1

if __name__ == "__main__":
    main()