import os
import math
import pychrono as chrono
import pychrono.sensor as sens

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA_DIR'], 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))

    
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    
    
    obj_mesh = chrono.ChObjFileShape()
    obj_mesh.SetFilename("model.obj")
    mesh_body.AddAsset(obj_mesh)
    system.Add(mesh_body)

    
    orbit_body = chrono.ChBody()
    orbit_body.SetMass(0)  
    system.Add(orbit_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100), chrono.ChColor(1, 1, 1), 500.0)

    
    time_step = 0.005
    update_rate = 1/time_step  
    image_width = 64
    image_height = 48
    fov = 70
    orbit_radius = 3.0
    orbit_height = 2.0
    orbit_speed = 0.5  

    
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

    
    noise_model = sens.ChNoiseNormal(0.0, 0.1)
    camera.AddNoiseModel(noise_model)

    
    camera.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera View"))

    manager.AddSensor(camera)

    
    time = 0.0
    while time < 10.0:
        
        new_time = time + time_step
        angle = orbit_speed * new_time
        x = orbit_radius * math.cos(angle)
        y = orbit_radius * math.sin(angle)
        pos = chrono.ChVectorD(x, y, orbit_height)
        
        
        look_frame = chrono.ChFrameD(pos)
        look_frame.LookAt(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1))
        
        orbit_body.SetPos(pos)
        orbit_body.SetRot(look_frame.GetRot())

        
        system.DoStepDynamics(time_step)
        
        
        manager.Update()

        
        if camera.GetMostRecentFrame():
            frame = camera.GetMostRecentFrame()
            print(f"\nTime: {new_time:.3f}")
            print(f"Camera Position: {pos.x():.2f}, {pos.y():.2f}, {pos.z():.2f}")
            print(f"Image Resolution: {image_width}x{image_height}")
            print(f"First pixel RGBA: {frame.GetRGBA8Data()[0][0]}")

        time = new_time

if __name__ == "__main__":
    main()