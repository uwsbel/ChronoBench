import os
import sys
import math
import numpy as np

import pychrono as chrono
import pychrono.sensor as sens






Vec3d = getattr(chrono, "ChVector3d", chrono.ChVectorD)
Vec3f = getattr(chrono, "ChVector3f", Vec3d)


def quat_from_angle_axis(angle, axis):
    if hasattr(chrono, "QuatFromAngleAxis"):
        return chrono.QuatFromAngleAxis(angle, axis)
    return chrono.Q_from_AngAxis(angle, axis)


def set_gravity(system, g):
    if hasattr(system, "SetGravitationalAcceleration"):
        system.SetGravitationalAcceleration(g)
    else:
        system.Set_G_acc(g)


def set_body_fixed(body, fixed=True):
    if hasattr(body, "SetFixed"):
        body.SetFixed(fixed)
    else:
        body.SetBodyFixed(fixed)


def add_visual_shape(body, shape):
    if hasattr(body, "AddVisualShape"):
        body.AddVisualShape(shape)
    else:
        body.AddAsset(shape)






def create_demo_obj(filename):
    
    obj_text = 
    with open(filename, "w") as f:
        f.write(obj_text)






def make_orbit_camera_pose(t, radius=4.0, height=1.5, angular_speed=0.5):
    
    theta = angular_speed * t

    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    z = height

    pos = Vec3d(x, y, z)

    
    yaw = theta + math.pi

    
    pitch = math.atan2(height, radius)

    q_yaw = quat_from_angle_axis(yaw, Vec3d(0, 0, 1))
    q_pitch = quat_from_angle_axis(pitch, Vec3d(0, 1, 0))

    rot = q_yaw * q_pitch

    return chrono.ChFrameD(pos, rot)






def main():
    
    
    

    step_size = 1.0 / 120.0
    end_time = 10.0

    camera_update_rate = 30.0
    image_width = 640
    image_height = 480
    camera_fov = math.radians(60.0)

    orbit_radius = 4.0
    orbit_height = 1.5
    orbit_angular_speed = 0.7

    
    
    

    if len(sys.argv) > 1:
        mesh_obj_path = sys.argv[1]
    else:
        mesh_obj_path = "demo_triangular_mesh.obj"
        if not os.path.exists(mesh_obj_path):
            create_demo_obj(mesh_obj_path)

    mesh_obj_path = os.path.abspath(mesh_obj_path)

    if not os.path.exists(mesh_obj_path):
        raise FileNotFoundError(f"OBJ mesh file not found: {mesh_obj_path}")

    print(f"Loading mesh: {mesh_obj_path}")

    
    
    

    system = chrono.ChSystemNSC()
    set_gravity(system, Vec3d(0, 0, -9.81))

    
    
    

    trimesh = chrono.ChTriangleMeshConnected()

    
    
    
    trimesh.LoadWavefrontMesh(mesh_obj_path, False, True)

    if hasattr(chrono, "ChVisualShapeTriangleMesh"):
        mesh_shape = chrono.ChVisualShapeTriangleMesh()
    else:
        mesh_shape = chrono.ChTriangleMeshShape()

    mesh_shape.SetMesh(trimesh)
    mesh_shape.SetName("Fixed OBJ Triangle Mesh")

    if hasattr(mesh_shape, "SetColor"):
        mesh_shape.SetColor(chrono.ChColor(0.75, 0.8, 0.95))

    if hasattr(mesh_shape, "SetMutable"):
        mesh_shape.SetMutable(False)

    if hasattr(mesh_shape, "SetBackfaceCull"):
        mesh_shape.SetBackfaceCull(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetName("fixed_mesh_body")
    set_body_fixed(mesh_body, True)
    mesh_body.SetPos(Vec3d(0, 0, 0))

    add_visual_shape(mesh_body, mesh_shape)

    system.Add(mesh_body)

    
    
    

    manager = sens.ChSensorManager(system)

    
    if hasattr(manager.scene, "SetAmbientLight"):
        manager.scene.SetAmbientLight(chrono.ChColor(0.35, 0.35, 0.35))

    manager.scene.AddPointLight(
        Vec3f(3.0, 4.0, 5.0),
        chrono.ChColor(1.0, 1.0, 1.0),
        20.0
    )

    manager.scene.AddPointLight(
        Vec3f(-4.0, -3.0, 4.0),
        chrono.ChColor(0.6, 0.7, 1.0),
        15.0
    )

    
    
    

    initial_camera_pose = make_orbit_camera_pose(
        0.0,
        radius=orbit_radius,
        height=orbit_height,
        angular_speed=orbit_angular_speed
    )

    camera = sens.ChCameraSensor(
        mesh_body,              
        camera_update_rate,     
        initial_camera_pose,    
        image_width,
        image_height,
        camera_fov
    )

    camera.SetName("orbiting_camera")
    camera.SetLag(0.0)
    camera.SetCollectionWindow(1.0 / camera_update_rate)

    
    
    
    
    

    
    camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.015))

    
    camera.PushFilter(
        sens.ChFilterVisualize(
            image_width,
            image_height,
            "Noisy orbiting camera image"
        )
    )

    
    camera.PushFilter(sens.ChFilterRGBA8Access())

    manager.AddSensor(camera)

    
    
    

    print("Starting simulation...")

    while system.GetChTime() < end_time:
        t = system.GetChTime()

        
        new_camera_pose = make_orbit_camera_pose(
            t,
            radius=orbit_radius,
            height=orbit_height,
            angular_speed=orbit_angular_speed
        )
        camera.SetOffsetPose(new_camera_pose)

        
        manager.Update()

        
        system.DoStepDynamics(step_size)

        
        rgba_buffer = camera.GetMostRecentRGBA8Buffer()

        if rgba_buffer is not None and rgba_buffer.HasData():
            rgba_data = rgba_buffer.GetRGBA8Data()
            rgba_array = np.asarray(rgba_data)

            
            
            shape = rgba_array.shape
            mean_val = float(np.mean(rgba_array))
            first_pixel = rgba_array[0, 0].tolist()

            print(
                f"t = {t:7.4f} s | "
                f"buffer shape = {shape} | "
                f"mean RGBA value = {mean_val:8.3f} | "
                f"first pixel = {first_pixel}"
            )
        else:
            print(f"t = {t:7.4f} s | camera buffer has no data yet")

    print("Simulation complete.")


if __name__ == "__main__":
    main()