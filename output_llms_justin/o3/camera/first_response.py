#!/usr/bin/env python3
# ---------------------------------------------------------------------
#  Fixed triangular mesh + orbiting camera sensor with visualisation,
#  noise filters and live buffer read-back.
# ---------------------------------------------------------------------

import math
import pychrono as chrono
import pychrono.sensor as sens            # Sensor module

# 1. ------------------------------------------------------------------
#    General set-up
chrono.SetChronoDataPath(chrono.GetChronoDataPath())        # so that Chrono can find data
system        = chrono.ChSystemSMC()                         # basic NSC or SMC works the same
time_step     = 1.0/240.0                                    # seconds
end_time      = 5.0                                          # run for a few seconds

# 2. ------------------------------------------------------------------
#    Fixed body that carries the triangular mesh read from an OBJ file
mesh          = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("my_mesh.obj", save_vertex_normals=True, save_uv=False)

mesh_shape    = chrono.ChVisualShapeTriangleMesh(mesh)
mesh_shape.SetBackfaceCull(True)

mesh_body     = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.AddVisualShape(mesh_shape)

system.Add(mesh_body)

# 3. ------------------------------------------------------------------
#    Sensor manager (handles all CUDA/OptiX work internally)
manager       = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(2,3,2),  chrono.ChColor(1,1,1), 10)
manager.scene.AddPointLight(chrono.ChVectorF(-2,3,-2),chrono.ChColor(1,1,1), 10)

# 4. ------------------------------------------------------------------
#    Camera sensor definition
cam_rate      = 30.0                                         # Hz
res_x, res_y  = 640, 480
fov           = 60                                           # degrees
initial_frame = chrono.ChFrameD(chrono.ChVectorD(3,1,0),     # start position
                                chrono.ChQuaternionD(1,0,0,0))

camera = sens.ChCameraSensor(mesh_body,                      # parent
                             cam_rate,                       # update rate [Hz]
                             initial_frame,                  # offset pose wrt parent body
                             res_x, res_y,                   # resolution
                             math.radians(fov))              # field of view

# --- Filters ---------------------------------------------------------
# Raw RGBA access (needed so we can read the image in Python)
access_rgba    = sens.ChFilterRGBA8Access()
camera.PushFilter(access_rgba)

# Add normal (independent) noise to each colour channel
camera.PushFilter(sens.ChFilterCameraNoiseNormal(0,          # mean
                                                 0.02,0.02,0.02)) # σR,σG,σB

# On-screen display   
camera.PushFilter(sens.ChFilterVisualize(res_x, res_y, "Orbiting camera"))

# Register the camera with the manager
manager.AddSensor(camera)

# 5. ------------------------------------------------------------------
#    Simple helper that re-point camera towards origin every iteration
def build_lookat_quat(from_pos, to=chrono.ChVectorD(0,0,0)):
    """
    Returns a quaternion that orients -Z (Chrono camera forward) from 'from_pos' to 'to'.
    """
    dir_z  = (to - from_pos)
    if dir_z.Length() < 1e-12:
        return chrono.ChQuaternionD(1,0,0,0)
    dir_z.Normalize()

    up     = chrono.ChVectorD(0,1,0)
    dir_x  = up.Cross(dir_z);  dir_x.Normalize()
    dir_y  = dir_z.Cross(dir_x)

    M      = chrono.ChMatrix33D()
    M.Set_A_Xdir(dir_x, dir_y, dir_z)      # columns = x,y,z directions
    return chrono.ChQuaternionD(M)

# 6. ------------------------------------------------------------------
#    Main time integration loop
omega         = 2*math.pi/5.0                               # one revolution in 5 s
step          = 0
print("Running simulation …")
while system.GetChTime() < end_time:
    # -------------------------------------------------
    # Update camera orbit (just change the sensor's offset pose)
    t          = system.GetChTime()
    radius     = 3.0
    cam_pos    = chrono.ChVectorD(radius*math.cos(omega*t), 1.0,
                                  radius*math.sin(omega*t))
    cam_rot    = build_lookat_quat(cam_pos)                 # points toward origin
    camera.SetOffsetPose(chrono.ChFrameD(cam_pos, cam_rot))

    # -------------------------------------------------
    # Dynamics and sensing
    system.DoStepDynamics(time_step)
    manager.Update()                                        # updates sensors
    manager.PostProcess()                                   # runs filter chains

    # -------------------------------------------------
    # Fetch image buffer and print a tiny piece of data
    if access_rgba.BufferIsReady():                         # ensures the GPU finished
        img = access_rgba.GetBuffer()                       # numpy array (uint8, H×W×4)
        # Print the first pixel (R,G,B,A) just as a demonstration
        r,g,b,a = img[0,0]
        print(f"[{step:04}]  t = {t:6.3f}  first-px = ({r:3},{g:3},{b:3},{a:3})")

    step += 1

print("Simulation finished.")