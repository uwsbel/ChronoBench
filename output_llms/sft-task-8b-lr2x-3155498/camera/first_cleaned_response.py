import argparse
import math
import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

STEP = 0.01
CAMERA_UPDATE_RATE = 10.0
CAMERA_RADIUS = 5.0
CAMERA_HEIGHT = 2.0
CAMERA_YAW = 0.0
CAMERA_PITCH = 0.0
CAMERA_FOV = 0.55
CAMERA_NEAR = 0.1
CAMERA_FAR = 100.0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
NOISE_TYPE = sens.ChSensorNoiseType.NONE
NOISE_MAGNITUDE = 0.001
NOISE_FREQ = 0.01
NOISE_PHASE = 0.0
VISUALIZE_IMAGES = True
VISUALIZE_NOISE = True
VISUALIZE_SENSORS = True


def add_camera_system(world, body, name, update_rate, radius, height, yaw, pitch, fov, near, far, width, height, noise_type, noise_magnitude, noise_frequency, noise_phase, visualize_images, visualize_noise, visualize_sensors):
    camera = sens.ChCameraSensor(body, update_rate, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))
    camera.SetName(name)
    camera.SetUpdateRate(update_rate)
    camera.SetOffset(chrono.ChFramed(chrono.ChVector3d(radius, height, 0), chrono.QUNIT))
    camera.SetDirection(chrono.ChVector3d(0, 0, 1))
    camera.SetUp(chrono.ChVector3d(0, 1, 0))
    camera.SetFOV(fov)
    camera.SetNearClip(near)
    camera.SetFarClip(far)
    camera.SetImageWidth(width)
    camera.SetImageHeight(height)
    camera.SetNoiseModel(noise_type)
    camera.SetNoiseMagnitude(noise_magnitude)
    camera.SetNoiseFrequency(noise_frequency)
    camera.SetNoisePhase(noise_phase)
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Image"))
    camera.PushFilter(sens.ChFilterRGBANoise(0.01, 0.01, 0.01, 0.01))
    camera.PushFilter(sens.ChFilterImageConvert())
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Noise"))
    camera.PushFilter(sens.ChFilterImageConvert())
    world.AddSensor(camera)
    return camera


def add_camera_visual(world, name, radius, height, yaw, pitch):
    visual = sens.ChVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
    visual.SetImageWidth(1280)
    visual.SetImageHeight(720)
    visual.SetClearColor(chrono.ChColor(0.1, 0.1, 0.1))
    world.AddSensor(visual)
    return visual


def add_camera_visualization_system(world, name, radius, height, yaw, pitch):
    visual = sens.ChCameraVisualSystem()
    visual.SetName(name)
    visual.SetUpdateRate(10)
    visual.SetCameraPosition(chrono.ChVector3d(radius, height, 0))
    visual.SetCameraDirection(chrono.ChVector3d(0, 0, 1))
    visual.SetCameraUp(chrono.ChVector3d(0, 1, 0))
    visual.SetCameraFOV(chrono.CH_PI / 4)
    visual.SetCameraNear(0.1)
    visual.SetCameraFar(100)
print("error happened with only start ```python")