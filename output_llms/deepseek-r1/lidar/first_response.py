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
    system.SetGravity(chrono.ChVectorD(0, 0, 0))

    # 2. Create fixed mesh body
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    # Load mesh from OBJ file
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))  # Replace with your OBJ file
    mesh_shape = chrono.ChTriangleMeshShape(mesh)
    mesh_shape.SetStatic(True)
    mesh_body.AddAsset(mesh_shape)
    system.Add(mesh_body)

    # 3. Create sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2, 2), chrono.ChColor(1, 1, 1), 100)

    # Lidar parameters
    lidar_offset = chrono.ChVectorD(0, 0.5, 0)  # Initial offset
    lidar_pos = chrono.ChFrameD(lidar_offset)    # Relative to mesh body
    lidar_update_rate = 5                        # Hz
    lidar_hres = 90                              # Horizontal samples
    lidar_vres = 45                              # Vertical samples
    lidar_hfov = 180                             # Horizontal FOV (deg)
    lidar_vfov = 30                              # Vertical FOV (deg)

    # Create lidar with noise model
    noise_model = sens.ChNoiseNormal(chrono.ChVectorD(0, 0, 0.01), chrono.ChVectorD(0, 0, 0.001))
    lidar = sens.ChLidarSensor(
        mesh_body,                # Parent body
        lidar_update_rate,        # Update rate
        lidar_pos,                # Offset pose
        lidar_hres,               # Horizontal resolution
        lidar_vres,               # Vertical resolution
        math.radians(lidar_hfov), # Horizontal FOV
        math.radians(lidar_vfov), # Vertical FOV
        0.1,                      # Max distance
        sens.ClipFrustum()        # Clipping method
    )
    lidar.SetName("Rotating Lidar")
    lidar.SetLag(0.1)
    lidar.SetCollectionWindow(0.02)
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterAccess())
    lidar.PushFilter(sens.ChFilterVisualize(lidar_hres, lidar_vres, "Lidar Data"))
    lidar.PushFilter(sens.ChFilterSave("lidar_data/"))
    lidar.PushFilter(sens.ChFilterNoise(noise_model))
    manager.AddSensor(lidar)

    # 4. Simulation loop
    orbit_radius = 3.0
    orbit_height = 1.0
    orbit_speed = 0.1  # rad/s
    time_step = 0.01
    total_time = 10.0

    for t in np.arange(0, total_time, time_step):
        # Update lidar position in orbit
        angle = orbit_speed * t
        new_x = orbit_radius * math.cos(angle)
        new_z = orbit_radius * math.sin(angle)
        lidar_offset = chrono.ChVectorD(new_x, orbit_height, new_z)
        lidar.SetOffsetPose(chrono.ChFrameD(lidar_offset))

        # Advance simulation
        manager.Update()
        system.DoStepDynamics(time_step)

        # Print lidar data
        buffer = lidar.GetMostRecentBuffer()
        if buffer.HasData():
            data = buffer.GetDepthData()
            print(f"Time: {t:.2f}s | Points: {len(data)}")
            if len(data) > 0:
                print(f"First point: Pos={data[0]} | Intensity={data[0].intensity}")

if __name__ == "__main__":
    main()