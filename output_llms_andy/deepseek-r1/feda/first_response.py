import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# =============================================================================
# Main simulation function
# =============================================================================

def main():
    # Set path to Chrono data directory
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

    # Simulation parameters
    step_size = 0.005
    render_fps = 50
    render_step_size = 1.0 / render_fps

    # Create Chrono system
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    # Initialize FEDA vehicle
    init_loc = chrono.ChVector3d(0, 0, 1.0)  # Start 1m above ground
    init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # Identity rotation
    contact_method = chrono.ChContactMethod_NSC
    vehicle = veh.FEDA(system)
    vehicle.SetInitPosition(chrono.ChFrame(init_loc, init_rot))
    vehicle.SetTireType(veh.TireModelType_RIGID)  # Use rigid tires
    vehicle.Initialize()

    # Set visualization mode to MESH for all vehicle parts
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create rigid terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(), 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowTitle("FEDA Vehicle Simulation")
    vis.SetWindowSize(1280, 720)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, -8, 2), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVector3d(-10, 10, 10), chrono.ChVector3d(0, 0, 0), 50, 5, 50, 50, 512)
    vis.EnableShadows()

    # Set camera to follow vehicle
    camera = irr.ChCamera()
    camera.SetPosition(chrono.ChVector3d(-3, 0, 2))
    camera.SetAimPoint(chrono.ChVector3d(0, 0, 1))
    vis.SetActiveCamera(camera)

    # Create interactive driver
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()
    step_number = 0
    time = 0

    while vis.Run():
        time = system.GetChTime()

        # Update driver inputs
        driver_inputs = driver.GetInputs()
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

        # Update visualization at specified FPS
        if step_number % int(render_step_size / step_size) == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Step the system
        system.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)
        step_number += 1

# =============================================================================
# Entry point
# =============================================================================
if __name__ == "__main__":
    main()