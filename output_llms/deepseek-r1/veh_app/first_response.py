import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # 1. Initialize PyChrono environment
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))
    system.SetSolverMaxIterations(100)

    # 2. Create rigid terrain
    terrain = veh.RigidTerrain(system)
    terrain_mat = chrono.ChMaterialSurfaceNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)
    terrain.Initialize(chrono.ChVectorD(0, 0, 0), 
                      chrono.ChVectorD(0, 0, 1), 
                      200, 200, 0.1, terrain_mat)
    terrain.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)

    # 3. Initialize Gator vehicle
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
    gator.Initialize()
    
    # Set visualization types for different parts
    gator.GetChassis().GetBody().SetVisualizationType(chrono.VisualizationType_PRIMITIVES)
    gator.GetSteering().GetVisualBody().SetVisualizationType(chrono.VisualizationType_PRIMITIVES)
    for wheel in gator.GetWheels():
        wheel.GetSpindle().SetVisualizationType(chrono.VisualizationType_MESH)

    # 4. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Gator Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 3, 1.5))
    vis.AddTypicalLights()

    # 5. Create interactive driver
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 6. Set up sensor manager
    sensor_manager = sens.ChSensorManager(system)
    sensor_manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 5), chrono.ChColor(1, 1, 1), 5.0)
    
    # Create and attach camera
    camera = sens.ChCameraSensor(
        gator.GetChassisBody(),  # parent body
        30,  # update rate in Hz
        chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0.5), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0))),  # offset pose
        1280,  # image width
        720,   # image height
        chrono.CH_C_PI / 3  # horizontal FOV
    )
    camera.SetName("Vehicle Camera")
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera View"))
    sensor_manager.AddSensor(camera)

    # 7. Simulation loop
    time_step = 0.005
    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        
        # Update vehicle and terrain
        gator.Synchronize(time, driver.GetInputs(), terrain)
        terrain.Synchronize(time)
        
        # Advance simulation
        driver.Advance(time_step)
        gator.Advance(time_step)
        system.DoStepDynamics(time_step)
        
        # Update sensors
        sensor_manager.Update()
        
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()