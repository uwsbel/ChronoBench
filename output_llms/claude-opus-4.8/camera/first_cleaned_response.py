import pychrono.core as chrono
import pychrono.sensor as sens
import math
import time






mphysicalSystem = chrono.ChSystemNSC()






mesh_filename = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")


trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(mesh_filename, False, True)


trimesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(1.0))


trimesh_shape = chrono.ChVisualShapeTriangleMesh()
trimesh_shape.SetMesh(trimesh)
trimesh_shape.SetName("Triangular Mesh")
trimesh_shape.SetMutable(False)


mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetFixed(True)  


mphysicalSystem.Add(mesh_body)





manager = sens.ChSensorManager(mphysicalSystem)


manager.scene.AddPointLight(
    chrono.ChVector3f(100, 100, 100),   
    chrono.ChColor(1, 1, 1),            
    1000                                
)






update_rate = 30.0


image_width = 1280
image_height = 720


fov = 1.408


lag = 0.0


exposure_time = 0.0






offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5, 0, 2),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0))
)


cam = sens.ChCameraSensor(
    mesh_body,        
    update_rate,      
    offset_pose,      
    image_width,      
    image_height,     
    fov               
)
cam.SetName("Orbiting Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)








cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))


cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera (with Noise)"))


cam.PushFilter(sens.ChFilterRGBA8Access())


manager.AddSensor(cam)






orbit_radius = 5.0
orbit_rate = 0.5        
orbit_height = 2.0


step_size = 1e-2
end_time = 40.0





ch_time = 0.0
render_time = 0.0

print("Starting simulation...\n")

while ch_time < end_time:

    
    
    cam_x = orbit_radius * math.cos(ch_time * orbit_rate)
    cam_y = orbit_radius * math.sin(ch_time * orbit_rate)

    new_pose = chrono.ChFramed(
        chrono.ChVector3d(cam_x, cam_y, orbit_height),
        
        chrono.QuatFromAngleAxis(ch_time * orbit_rate + math.pi,
                                 chrono.ChVector3d(0, 0, 1))
    )
    cam.SetOffsetPose(new_pose)

    
    manager.Update()

    
    rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
    if rgba8_buffer.HasData():
        rgba8_data = rgba8_buffer.GetRGBA8Data()
        print(f"[t = {ch_time:6.2f} s] "
              f"Camera buffer received: "
              f"{rgba8_buffer.Width} x {rgba8_buffer.Height} | "
              f"First pixel RGBA = "
              f"({int(rgba8_data[0,0,0])}, {int(rgba8_data[0,0,1])}, "
              f"{int(rgba8_data[0,0,2])}, {int(rgba8_data[0,0,3])})")
    else:
        print(f"[t = {ch_time:6.2f} s] No camera data yet...")

    
    mphysicalSystem.DoStepDynamics(step_size)

    
    ch_time = mphysicalSystem.GetChTime()

print("\nSimulation complete.")