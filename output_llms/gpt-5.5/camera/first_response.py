# pychrono_mesh_camera_orbit.py
#
# PyChrono simulation:
# - Loads a triangular Wavefront OBJ mesh as a fixed visual body
# - Attaches a camera sensor to that body
# - Uses a ChSensorManager with lighting
# - Applies camera noise and visualization filters
# - Dynamically orbits the camera around the mesh
# - Prints camera RGBA buffer information at every simulation step
#
# Usage:
#   python pychrono_mesh_camera_orbit.py path/to/mesh.obj
#
# If no OBJ path is supplied, the script creates a small demo OBJ mesh.

import os
import sys
import math
import numpy as np

import pychrono as chrono
import pychrono.sensor as sens


# ---------------------------------------------------------------------
# Compatibility helpers for different PyChrono versions
# ---------------------------------------------------------------------

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


# ---------------------------------------------------------------------
# Create a fallback OBJ mesh if the user does not provide one
# ---------------------------------------------------------------------

def create_demo_obj(filename):
    """
    Create a simple triangular pyramid mesh.
    """
    obj_text = """# Simple triangular mesh demo OBJ
v  0.0  0.0  1.0
v -1.0 -1.0  0.0
v  1.0 -1.0  0.0
v  1.0  1.0  0.0
v -1.0  1.0  0.0

f 1 2 3
f 1 3 4
f 1 4 5
f 1 5 2
f 2 5 4
f 2 4 3
"""
    with open(filename, "w") as f:
        f.write(obj_text)


# ---------------------------------------------------------------------
# Camera orbit pose
# ---------------------------------------------------------------------

def make_orbit_camera_pose(t, radius=4.0, height=1.5, angular_speed=0.5):
    """
    Return a ChFrameD describing a camera orbiting around the origin.

    Chrono camera sensors use the local +X axis as the optical axis.
    This pose keeps +X approximately pointed toward the origin.
    """
    theta = angular_speed * t

    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    z = height

    pos = Vec3d(x, y, z)

    # Yaw so camera local +X points toward the origin in the XY plane.
    yaw = theta + math.pi

    # Pitch down toward the origin.
    pitch = math.atan2(height, radius)

    q_yaw = quat_from_angle_axis(yaw, Vec3d(0, 0, 1))
    q_pitch = quat_from_angle_axis(pitch, Vec3d(0, 1, 0))

    rot = q_yaw * q_pitch

    return chrono.ChFrameD(pos, rot)


# ---------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------

def main():
    # -------------------------------------------------------------
    # Simulation parameters
    # -------------------------------------------------------------

    step_size = 1.0 / 120.0
    end_time = 10.0

    camera_update_rate = 30.0
    image_width = 640
    image_height = 480
    camera_fov = math.radians(60.0)

    orbit_radius = 4.0
    orbit_height = 1.5
    orbit_angular_speed = 0.7

    # -------------------------------------------------------------
    # Mesh path
    # -------------------------------------------------------------

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

    # -------------------------------------------------------------
    # Initialize Chrono physical system
    # -------------------------------------------------------------

    system = chrono.ChSystemNSC()
    set_gravity(system, Vec3d(0, 0, -9.81))

    # -------------------------------------------------------------
    # Load Wavefront OBJ mesh and create fixed body
    # -------------------------------------------------------------

    trimesh = chrono.ChTriangleMeshConnected()

    # Arguments:
    #   load_normals = False
    #   load_uv      = True
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

    # -------------------------------------------------------------
    # Create sensor manager
    # -------------------------------------------------------------

    manager = sens.ChSensorManager(system)

    # Lighting for camera rendering
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

    # -------------------------------------------------------------
    # Create camera sensor attached to the fixed mesh body
    # -------------------------------------------------------------

    initial_camera_pose = make_orbit_camera_pose(
        0.0,
        radius=orbit_radius,
        height=orbit_height,
        angular_speed=orbit_angular_speed
    )

    camera = sens.ChCameraSensor(
        mesh_body,              # parent body
        camera_update_rate,     # update rate in Hz
        initial_camera_pose,    # offset pose relative to parent body
        image_width,
        image_height,
        camera_fov
    )

    camera.SetName("orbiting_camera")
    camera.SetLag(0.0)
    camera.SetCollectionWindow(1.0 / camera_update_rate)

    # -------------------------------------------------------------
    # Camera filters
    # -------------------------------------------------------------
    # Filter order matters. The RGBA access filter should generally
    # come after filters whose result you want to access.

    # Add constant normal-distribution camera noise.
    camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.015))

    # Display the noisy camera image in a visualization window.
    camera.PushFilter(
        sens.ChFilterVisualize(
            image_width,
            image_height,
            "Noisy orbiting camera image"
        )
    )

    # Allow CPU-side access to the final RGBA8 image buffer.
    camera.PushFilter(sens.ChFilterRGBA8Access())

    manager.AddSensor(camera)

    # -------------------------------------------------------------
    # Simulation loop
    # -------------------------------------------------------------

    print("Starting simulation...")

    while system.GetChTime() < end_time:
        t = system.GetChTime()

        # Dynamically move the camera in an orbit around the fixed mesh.
        new_camera_pose = make_orbit_camera_pose(
            t,
            radius=orbit_radius,
            height=orbit_height,
            angular_speed=orbit_angular_speed
        )
        camera.SetOffsetPose(new_camera_pose)

        # Update sensors.
        manager.Update()

        # Advance dynamics.
        system.DoStepDynamics(step_size)

        # Access the camera image buffer.
        rgba_buffer = camera.GetMostRecentRGBA8Buffer()

        if rgba_buffer is not None and rgba_buffer.HasData():
            rgba_data = rgba_buffer.GetRGBA8Data()
            rgba_array = np.asarray(rgba_data)

            # Example buffer diagnostics.
            # Shape is usually height x width x 4.
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