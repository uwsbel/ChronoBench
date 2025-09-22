import os
import math
import pychrono as chrono
import pychrono.irrlicht as irrlicht
import pychrono.sensor as sens

def main():
    # 1. Initialize simulation system
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, 0))

    # 2. Create ground body and mesh
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    # Load mesh from OBJ file
    obj_path = os.path.join(chrono.GetChronoDataPath(), "models", "cube.obj")
    mesh = chrono.ChVisualShapeModelFile()
    mesh.SetFilename(obj_path)
    ground.AddVisualShape(mesh)

    # 3. Create sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 1000)

    # Camera parameters
    orbit_radius = 3.0
    orbit_height = 1.0
    orbit_speed = 0.1
    camera_update_rate = 30
    camera_resolution = (1280, 720)
    camera_fov = 1.0  # Field of view (radians)

    # 4. Create camera sensor with noise and filters
    camera = sens.ChCameraSensor(
        ground,                     # Attached to ground body
        camera_update_rate,         # Update rate
        chrono.ChFramed(            # Initial pose (will be updated)
            chrono.ChVector3d(orbit_radius, 0, orbit_height),
            chrono.QuatFromAngleZ(0)
        ),
        camera_resolution[0],       # Image width
        camera_resolution[1],       # Image height
        camera_fov                  # Field of view
    )
    camera.SetName("Orbiting Camera")
    camera.SetLag(0.1)  # Simulate processing delay

    # Add noise model
    noise_model = sens.AddNoiseModel(sens.NoiseModelType.NORMAL, 0.0, 0.2)
    camera.PushFilter(sens.ChFilterRGBA8Noise(noise_model))

    # Add visualizations
    camera.PushFilter(sens.ChFilterVisualize(camera_resolution[0], camera_resolution[1], "Camera Output"))
    camera.PushFilter(sens.ChFilterRGBA8Access())

    # Add camera to manager
    manager.AddSensor(camera)

    # 5. Create Irrlicht visualization
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Orbiting Camera Simulation")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 4, 2), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    # 6. Simulation loop
    time_step = 0.001
    step = 0
    while vis.Run():
        # Update camera orbit position
        angle = orbit_speed * system.GetChTime()
        cam_pos = chrono.ChVector3d(
            orbit_radius * math.cos(angle),
            orbit_radius * math.sin(angle),
            orbit_height
        )
        camera.SetOffsetPose(chrono.ChFramed(
            cam_pos,
            chrono.QuatFromAngleAxis(-angle, chrono.ChVector3d(0, 0, 1))
        ))

        # Update sensor manager
        manager.Update()

        # Print camera buffer data every 100 steps
        if step % 100 == 0:
            buffer = camera.GetMostRecentRGBA8Buffer()
            if buffer.HasData():
                print(f"Time: {system.GetChTime():.3f}")
                print(f"Camera position: {cam_pos}")
                print(f"Buffer size: {buffer.GetWidth()}x{buffer.GetHeight()}")
                print(f"First pixel RGBA: {buffer.GetRGBA8Data()[0][0]}")

        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Advance simulation
        system.DoStepDynamics(time_step)
        step += 1

if __name__ == "__main__":
    main()