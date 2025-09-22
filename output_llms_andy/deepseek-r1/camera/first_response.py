#!/usr/bin/env python3
import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import os
import math

def main():
    # 1. Initialize environment
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, 0))

    # 2. Create ground and mesh body
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    # Load mesh from OBJ file
    mesh_path = chrono.GetChronoDataFile("models/cube.obj")  # Replace with your OBJ path
    mesh = chrono.ChVisualShapeModelFile(mesh_path)
    ground.AddVisualShape(mesh)

    # 3. Create camera sensor system
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2, 2), chrono.ChColor(1, 1, 1), 500)

    # Camera orbit parameters
    orbit_radius = 3.0
    orbit_height = 1.0
    angular_speed = 0.1  # rad/s

    # Create camera body and initialize position
    camera_body = chrono.ChBody()
    camera_body.SetPos(chrono.ChVector3d(orbit_radius, 0, orbit_height))
    system.Add(camera_body)

    # Camera settings
    offset_pose = chrono.ChFramed()
    camera = sens.ChCameraSensor(
        camera_body,          # parent body
        0.1,                  # update rate
        offset_pose,          # offset pose
        1280,                 # image width
        720,                  # image height
        chrono.CH_PI / 3      # FOV
    )
    camera.SetName("Orbiting Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Output"))
    camera.PushFilter(sens.ChFilterNoiseNormal(0.0, 0.02))  # Gaussian noise

    manager.AddSensor(camera)

    # 4. Simulation loop
    step_size = 0.01
    time = 0
    while time < 10:
        # Update camera orbit position
        angle = angular_speed * time
        cam_x = orbit_radius * math.cos(angle)
        cam_y = orbit_radius * math.sin(angle)
        camera_body.SetPos(chrono.ChVector3d(cam_x, cam_y, orbit_height))
        camera_body.SetRot(chrono.QuatFromAngleAxis(-angle, chrono.ChVector3d(0, 0, 1)))

        # Update sensors
        manager.Update()

        # Access and print camera buffer data
        rgba8_data = camera.GetMostRecentRGBA8Buffer()
        if rgba8_data.HasData():
            buf = rgba8_data.GetRGBA8Data()
            print(f"Time: {time:.2f}s | Camera buffer: "
                  f"Avg R: {np.mean(buf[:,:,0]):.1f}, "
                  f"G: {np.mean(buf[:,:,1]):.1f}, "
                  f"B: {np.mean(buf[:,:,2]):.1f}, "
                  f"Size: {buf.shape}")

        # Advance simulation
        system.DoStepDynamics(step_size)
        time += step_size

if __name__ == "__main__":
    main()