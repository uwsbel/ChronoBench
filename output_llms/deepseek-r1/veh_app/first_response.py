import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # ===== 1. Initialize PyChrono environment =====
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVectorD(0, -9.81, 0))
    system.SetSolverMaxIterations(100)

    # ===== 2. Create and initialize Gator vehicle =====
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 1, 0)))
    gator.Initialize()
    
    # Set visualization modes for different parts
    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    # ===== 3. Create rigid terrain =====
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 200, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    terrain.Initialize()

    # ===== 4. Create interactive driver system =====
    driver = veh.InteractiveDriverIRR(gator.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)

    # ===== 5. Create Irrlicht visualization =====
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Gator Vehicle Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(8, 2, 0), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(-5, 8, -5), chrono.ChVectorD(0, 0, 0), 50, 3, 50, 40, 512)

    # ===== 6. Setup sensor manager =====
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 0), chrono.ChColor(0.8, 0.8, 0.8), 5.0)
    manager.scene.AddPointLight(chrono.ChVectorF(9, 3, 6), chrono.ChColor(0.7, 0.2, 0.2), 4.0)

    # Create camera sensor attached to chassis
    camera = sens.ChCameraSensor(
        gator.GetChassisBody(),  # parent body
        30,                      # scan rate (Hz)
        chrono.ChFrameD(chrono.ChVectorD(1, 1.5, -0.3), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0))),  # offset pose
        1280,                    # image width
        720,                     # image height
        chrono.CH_C_PI / 3       # FOV (60 deg)
    )
    camera.SetName("Vehicle Camera")
    camera.PushFilter(sens.ChFilterRGBA8())
    manager.AddSensor(camera)

    # ===== 7. Simulation loop =====
    step_size = 0.002
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver_inputs = driver.GetInputs()
        driver.Synchronize(time)
        
        # Update terrain
        terrain.Synchronize(time)
        
        # Update vehicle systems
        gator.Synchronize(time, driver_inputs, terrain)
        
        # Update sensor manager
        manager.Update()
        
        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)

if __name__ == "__main__":
    main()