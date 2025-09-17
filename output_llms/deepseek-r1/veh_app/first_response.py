#!/usr/bin/env python3
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # 1. Initialize environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.SetSolverMaxIterations(150)

    # 2. Create terrain
    terrain = veh.RigidTerrain(system)
    patch_mat = chrono.ChMaterialSurfaceNSC()
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), 100, 100)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # 3. Initialize Gator vehicle
    gator = veh.Gator(system)
    gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
    gator.Initialize()
    
    # Set visualization types for vehicle parts
    gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
    gator.SetTireVisualizationType(veh.VisualizationType_MESH)

    # 4. Create and initialize interactive driver
    driver = veh.ChInteractiveDriverIRR(gator.GetVehicle())
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.06)
    driver.Initialize()

    # 5. Create Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Gator Vehicle Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, 2, 10, 40, 512)

    # 6. Initialize sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 0), chrono.ChColor(1.0, 1.0, 1.0), 5.0)

    # Camera sensor attached to chassis
    camera = sens.ChCameraSensor(
        gator.GetChassisBody(),  # parent body
        30,                     # update rate
        chrono.ChFrameD(chrono.ChVectorD(0.5, 1.0, -0.3), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0,1,0))),
        1280,                   # image width
        720,                    # image height
        chrono.CH_C_PI / 3      # FOV
    )
    camera.SetName("Vehicle Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    # 7. Simulation loop
    step_size = 0.002
    render_steps = 1.0 / 60  # Render at ~60 FPS

    while vis.Run():
        time = system.GetChTime()
        
        # Update driver inputs
        driver.Synchronize(time)
        # Update terrain
        terrain.Synchronize(time)
        # Update vehicle
        gator.Synchronize(time, driver.GetInputs(), terrain)
        # Update sensors
        manager.Update()
        
        # Advance simulation
        driver.Advance(step_size)
        terrain.Advance(step_size)
        gator.Advance(step_size)
        system.DoStepDynamics(step_size)
        
        # Render scene
        if system.GetChTime() >= render_steps:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_steps += 1.0 / 60

if __name__ == "__main__":
    main()