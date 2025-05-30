import pychrono as chrono
import pychrono.sensor as sens
import math
import os

# Simulation parameters
SIM_TIME = 10.0
TIME_STEP = 0.001
ORBIT_RADIUS = 5.0
ANGULAR_SPEED = 0.5  # rad/s

# Lidar parameters
UPDATE_RATE = 30  # Hz
HORIZ_SAMPLES = 16
VERT_SAMPLES = 1
HORIZ_FOV = 2 * math.pi  # 360 degrees
VERT_FOV = math.pi / 18  # 10 degrees

def main():
    # Create Chrono system
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))

    # Create fixed mesh body
    mesh_body = chrono.ChBodyEasyMesh(
        "path/to/mesh.obj",  # Replace with actual path
        1000,                # Density
        True,                # Compute mass
        True,                # Visualization
        False                # Collision
    )
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(mesh_body)

    # Create sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorD(2, 2.5, 0), chrono.ChColor(1, 1, 1), 500)

    # Initialize lidar position
    initial_pos = chrono.ChVectorD(ORBIT_RADIUS, 0, 0)
    initial_rot = chrono.Q_from_AngZ(math.pi)
    offset_pose = chrono.ChFrameD(initial_pos, initial_rot)

    # Create lidar sensor
    lidar = sens.ChLidarSensor(
        mesh_body,
        UPDATE_RATE,
        offset_pose,
        HORIZ_SAMPLES,
        VERT_SAMPLES,
        HORIZ_FOV,
        VERT_FOV,
        VERT_FOV/2,
        0.0,
        1.0/UPDATE_RATE
    )
    lidar.SetName("Orbiting Lidar")
    
    # Add noise model
    lidar.AddNoiseModel(sens.ChNoiseNormalDist(0, 0.01))

    # Add processing filters
    lidar.PushFilter(sens.ChFilterAccess())
    lidar.PushFilter(sens.ChFilterVisualize(HORIZ_SAMPLES, VERT_SAMPLES, "Lidar Data"))
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterSavePtCloud("lidar_data"))

    manager.AddSensor(lidar)

    # Simulation loop
    current_time = 0.0
    while current_time < SIM_TIME:
        # Update lidar orbit position
        theta = ANGULAR_SPEED * current_time
        new_pos = chrono.ChVectorD(
            ORBIT_RADIUS * math.cos(theta),
            ORBIT_RADIUS * math.sin(theta),
            0
        )
        new_rot = chrono.Q_from_AngZ(theta + math.pi)
        lidar.SetOffsetPose(chrono.ChFrameD(new_pos, new_rot))

        # Update sensors
        manager.Update()

        # Access and print lidar data
        buffer = lidar.GetMostRecentBuffer()
        if buffer and buffer.HasData():
            print(f"\nTime: {current_time:.2f}")
            print(f"Lidar Position: {new_pos}")
            print(f"Points captured: {buffer.Width * buffer.Height}")
            
            # Print first 3 points (if available)
            pc = buffer.GetPointCloud()
            if pc:
                for i in range(min(3, len(pc))):
                    print(f"Point {i}: {pc[i]}")

        # Advance simulation
        system.DoStepDynamics(TIME_STEP)
        current_time += TIME_STEP

if __name__ == "__main__":
    main()