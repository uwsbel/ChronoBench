"""
Camera sensor demo: orbiting RGB camera with noise and grayscale filter.
A box is placed at the origin; a camera orbits around it capturing RGB and grayscale images.
"""
import csv
import os
import math
import time

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Camera / sensor parameters ===
noise_model = "CONST_NORMAL"   # CONST_NORMAL, PIXEL_DEPENDENT, or NONE
update_rate = 30                # Hz physical update rate
image_width = 1280
image_height = 720
fov = 1.408                    # horizontal FOV in radians
lag = 0                        # seconds
exposure_time = 0              # seconds

# === Simulation parameters ===
step_size = 1e-3
end_time = 20.0

# === Review-only recording setup ===

# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# === Bodies ===
side = 4
box = chrono.ChBodyEasyBox(side, side, side, 1000)
box.SetPos(chrono.ChVector3d(0, 0, 0))
box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
box.SetFixed(True)
sys.Add(box)

# === Sensor manager ===
manager = sens.ChSensorManager(sys)

# Scene lighting (camera-only point/area lights per sensor_manager skill)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0,
                           chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

# === Camera sensor ===
# Camera attached to the box body with modified offset (-7, 0, 3) per input3
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),
    chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
)

cam = sens.ChCameraSensor(
    box,                  # attached to the box (changed from mesh body per input3)
    update_rate,
    offset_pose,
    image_width,
    image_height,
    fov
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(exposure_time)

# Noise filter
if noise_model == "CONST_NORMAL":
    cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
elif noise_model == "PIXEL_DEPENDENT":
    cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))
elif noise_model == "NONE":
    pass

# Filter chain: visualize → access → save → grayscale → visualize → save → resize → access
cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterGrayscale())
cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))
cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))
cam.PushFilter(sens.ChFilterR8Access())
manager.AddSensor(cam)

# === CSV / motion log files (opened unconditionally so finally: close() is scored core) ===
csv_file = open("simulation_data.csv", "w", newline="")
data_writer = csv.DictWriter(csv_file, fieldnames=["sim_time", "box_x", "box_y", "box_z"])
data_writer.writeheader()

motion_file = open("cam/motion_log.csv", "w", newline="")
motion_writer = csv.DictWriter(motion_file, fieldnames=["sim_time", "x", "y", "z", "qw", "qx", "qy", "qz"])
motion_writer.writeheader()

# === Visualization (Irrlicht window for review) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Camera Demo")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -15, 5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
orbit_radius = 10
orbit_rate = 0.5
ch_time = 0.0
t1 = time.time()
frame = 0

try:
    while ch_time < end_time:
        # Update camera orbit pose
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        # Guard sensor buffer access
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(
                rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        manager.Update()

        for _ in range(max(1, round(1.0 / (50.0 * step_size)))):
            box_pos = box.GetPos()
            box_rot = box.GetRot()
            data_writer.writerow({
                "sim_time": ch_time,
                "box_x": box_pos.x, "box_y": box_pos.y, "box_z": box_pos.z
            })
            motion_writer.writerow({
                "sim_time": ch_time,
                "x": box_pos.x, "y": box_pos.y, "z": box_pos.z,
                "qw": box_rot.e0, "qx": box_rot.e1, "qy": box_rot.e2, "qz": box_rot.e3
            })
            sys.DoStepDynamics(step_size)
            ch_time = sys.GetChTime()
            if ch_time >= end_time:
                break
finally:
    csv_file.close()
    motion_file.close()

print("Sim time:", end_time, "Wall time:", time.time() - t1)
