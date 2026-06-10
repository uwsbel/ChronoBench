import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

sys = chrono.ChSystemNSC()                                          # NSC system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # contact/collision scene
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))        # no gravity (static sensing demo)

side = 2.0                                                          # box edge length (m)
mat = chrono.ChContactMaterialNSC()                                # default contact material

box = chrono.ChBodyEasyBox(side, side, side, 1000, True, True, mat) # box: side^3, density 1000
box.SetPos(chrono.ChVector3d(0, 0, 0))                             # box at the world origin
box.SetFixed(True)                                                 # box is the static sensing target
box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # box texture
sys.Add(box)                                                       # add the box to the system

manager = sens.ChSensorManager(sys)                                # OptiX sensor manager
intensity = 1.0                                                    # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)   # point light
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)   # point light

offset_pose = chrono.ChFramed(                                     # camera offset on the box frame
    chrono.ChVector3d(-7, 0, 3),                                   # offset position (-7, 0, 3)
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),     # slight downward tilt about +Y
)

cam = sens.ChCameraSensor(
    box,                                                          # attach the camera to the box
    30,                                                           # update_rate (Hz) — physical rate
    offset_pose,                                                  # offset pose on the box
    1280, 720,                                                    # image width, height
    1.408,                                                        # horizontal FOV (rad)
)
cam.SetName("Camera Sensor")                                      # sensor name
cam.SetLag(0)                                                     # lag = 0
cam.SetCollectionWindow(0)                                        # exposure/collection window = 0
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to the RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                    # save color PNG frames
manager.AddSensor(cam)                                            # register the camera

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht review window
vis.AttachSystem(sys)                                             # attach the system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                 # Z-up camera
vis.SetWindowSize(1280, 720)                                      # window size
vis.SetWindowTitle("Camera Sensor Demo")                         # window title
vis.Initialize()                                                 # init device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png")) # logo
vis.AddSkyBox()                                                  # sky box
vis.AddCamera(chrono.ChVector3d(-8, 0, 4), chrono.ChVector3d(0, 0, 0))  # window camera
vis.AddTypicalLights()                                           # standard lights

time_step = 1e-3                                                 # integration step (s)
sim_end = 5.0                                                    # simulation duration (s)
render_fps = 50.0                                                # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        manager.Update()                                        # pump sensors every physics step
        sys.DoStepDynamics(time_step)                           # advance one step
        buf = cam.GetMostRecentRGBA8Buffer()                   # most recent camera buffer
        if buf.HasData():                                      # only after the first sensor tick
            print('Buffer received. Resolution: {0}x{1}'.format(buf.Width, buf.Height))
        if sys.GetChTime() >= sim_end:
            break
