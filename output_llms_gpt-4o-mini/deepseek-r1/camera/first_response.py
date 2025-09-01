import os
import math
import pychrono as chrono
import pychrono.sensor as sens

def main():
    # Initialize Chrono system
    chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))

    # Create fixed triangular mesh body
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    # Load mesh from OBJ file (replace with your actual file path)
    obj_mesh = chrono.ChObjFileShape()
    obj_mesh.SetFilename("model.obj")
    mesh_body.AddAsset(obj_mesh)
    system.Add(mesh_body)

    # Create orbiting body for camera
    orbit_body = chrono.ChBody()
    orbit_body.SetMass(0)  # Make kinematic
    system.Add(orbit_body)

    # Sensor manager
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)

    # Camera parameters
    time_step = 0.005
    update_rate = 1/time_step  # Match simulation steps
    image_width = 64
    image_height = 48
    fov = 70
    orbit_radius = 3.0
    orbit_height = 2.0
    orbit_speed = 0.5  # Radians per second

    # Create camera sensor
    camera = sens.ChCameraSensor(
        orbit_body,
        update_rate,
        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))),
        image_width,
        image_height,
        fov
    )
    camera.SetName("Orbital Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0)

    # Add noise model
    noise_model = sens.ChNoiseNormal(0.0, 0.1)
    camera.AddNoiseModel(noise_model)

    # Enable visualization
    camera.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera View"))

    manager.AddSensor(camera)

    # Simulation loop
    time = 0.0
    while time < 10.0:
        # Update orbital position and orientation
        new_time = time + time_step
        angle = orbit_speed * new_time
        x = orbit_radius * math.cos(angle)
        y = orbit_radius * math.sin(angle)
        pos = chrono.ChVectorD(x, y, orbit_height)
        
        # Calculate look-at rotation
        look_frame = chrono.ChFrameD(pos)
        look_frame.LookAt(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
        
        orbit_body.SetPos(pos)
        orbit_body.SetRot(look_frame.GetRot())

        # Simulation step
        system.DoStepDynamics(time_step)
        
        # Update sensors
        manager.Update()

        # Print camera data
        if camera.GetMostRecentFrame():
            frame = camera.GetMostRecentFrame()
            print(f"\nTime: {new_time:.3f}")
            print(f"Camera Position: {pos.x():.2f}, {pos.y():.2f}, {pos.z():.2f}")
            print(f"Image Resolution: {image_width}x{image_height}")
            print(f"First pixel RGBA: {frame.GetRGBA8Data()[0][0]}")

        time = new_time

if __name__ == "__main__":
    main()