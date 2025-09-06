import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # =============================================================================
    # Initialize Simulation Environment
    # =============================================================================
    # Create Chrono system with SMC contact method
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    # Set data path for Chrono assets
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # =============================================================================
    # Create Rigid Terrain
    # =============================================================================
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChContactMaterialSMC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
    terrain.Initialize()

    # =============================================================================
    # Create and Initialize Gator Vehicle
    # =============================================================================
    gator = veh.Gator(system)
    gator.SetContactMethod(chrono.ChContactMethod_SMC)
    gator.SetChassisFixed(False)
    gator.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.0), chrono.QUNIT))
    gator.Initialize()

    # Configure vehicle visualization
    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Get vehicle systems for later reference
    vehicle = gator.GetVehicle()
    powertrain = gator.GetPowertrain()

    # =============================================================================
    # Create Interactive Driver System
    # =============================================================================
    driver = veh.ChInteractiveDriverIRR(vehicle)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # =============================================================================
    # Create Sensor System
    # =============================================================================
    # Sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(1.0, 1.0, 1.0), 500)
    
    # Camera sensor attached to chassis
    camera = sens.ChCameraSensor(
        gator.GetChassisBody(),  # parent body
        30,                      # update rate (Hz)
        chrono.ChFramed(chrono.ChVector3d(2, 0, 1.5), chrono.QuatFromAngleY(0.2)),  # offset pose
        1280,                    # image width
        720,                     # image height
        chrono.CH_PI / 3         # FOV
    )
    camera.SetName("Chassis Camera")
    camera.PushFilter(sens.ChFilterRGBA8())
    manager.AddSensor(camera)

    # =============================================================================
    # Create Visualization System
    # =============================================================================
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Gator Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(8, 8, 1.5))
    vis.AddTypicalLights()
    vis.AddGrid(0.5, 0.5, 12, 12, chrono.ChCoordsysd(), chrono.ChColor(0.2, 0.2, 0.2))

    # =============================================================================
    # Simulation Loop
    # =============================================================================
    step_size = 0.005
    render_steps = 1.0 / 50  # Render every 20ms

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update terrain
        terrain.Synchronize(time)
        
        # Update vehicle systems
        vehicle.Synchronize(time, driver.GetInputs(), terrain)
        powertrain.Synchronize(time, driver.GetInputs(), vehicle.GetDriveshaftSpeed())
        
        # Update sensor manager
        manager.Update()
        
        # Advance simulation
        system.DoStepDynamics(step_size)
        
        # Update visualization
        if time % render_steps < step_size:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            vis.Synchronize(driver.GetInputModeAsString(), driver.GetInputs())
    
    return 0

if __name__ == "__main__":
    main()