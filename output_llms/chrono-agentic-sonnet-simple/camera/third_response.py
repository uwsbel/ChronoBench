import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# simulation parameters
side = 1.0                                                           # box side length (m)
time_step = 1e-3                                                     # physics step size (s)
sim_end = 20.0                                                       # simulation end time (s)
render_fps = 30.0                                                    # render rate
render_every = max(1, round(1.0 / (render_fps * time_step)))        # render cadence (untagged)

# create physical system
sys = chrono.ChSystemNSC()                                           # NSC contact
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g = 9.81 m/s^2
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # bullet collision

# contact material for box and ground
mat = chrono.ChContactMaterialNSC()                                  # NSC material

# ground body (flat plane)
ground = chrono.ChBodyEasyBox(20, 20, 0.5, 1000, True, True, mat)  # large flat ground
ground.SetPos(chrono.ChVector3d(0, 0, -0.25))                       # just below z=0
ground.SetFixed(True)                                               # fixed in world
sys.AddBody(ground)                                                  # add to system

# box body (the protagonist — replaces the mesh)
box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True, mat)  # cube
box_body.SetPos(chrono.ChVector3d(0, 0, side / 2.0))               # sit on ground
sys.AddBody(box_body)                                               # add to system

# box texture (visual only)
box_body.GetVisualModel().GetShape(0).SetTexture(                   # apply texture to box
    chrono.GetChronoDataPath() + "textures/cubetexture_bluewhite.png"
)

# Irrlicht visualization — Initialize FIRST, then add scene elements
vis = chronoirr.ChVisualSystemIrrlicht()                            # create Irrlicht vis
vis.SetWindowSize(1280, 720)                                        # window resolution
vis.SetWindowTitle("Camera Sensor Demo — Box")                      # window title
vis.Initialize()                                                    # Initialize first
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png") # logo
vis.AddSkyBox()                                                     # skybox
vis.AddCamera(chrono.ChVector3d(5, 5, 3), chrono.ChVector3d(0, 0, 0))  # Irrlicht camera
vis.AddTypicalLights()                                              # lighting
vis.AttachSystem(sys)                                               # link to system

# sensor manager + scene lighting for OptiX camera
manager = sens.ChSensorManager(sys)                                 # sensor manager
intensity = 1.0
manager.scene.AddPointLight(                                        # key light above scene
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                        # fill light
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# camera sensor attached to the box — offset pose changed to (-7, 0, 3)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-7, 0, 3),                                    # offset from box body
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),     # tilt angle (rad) about Y
)

cam = sens.ChCameraSensor(
    box_body,                                                       # attach to box (not mesh)
    30,                                                             # update_rate 30 Hz
    offset_pose,                                                    # offset pose on the body
    1280, 720,                                                      # width, height
    1.408,                                                          # horizontal FOV (rad)
)
cam.SetName("Camera Sensor")
cam.SetLag(0)                                                       # zero lag
cam.SetCollectionWindow(0)                                          # zero exposure window

# filter chain — ORDER MATTERS; each Save captures buffer at its position
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "RGB Camera"))    # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                         # host access RGBA8
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                      # SAVE color PNGs (scored)
cam.PushFilter(sens.ChFilterGrayscale())                           # convert to grayscale
cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale"))     # grayscale preview
cam.PushFilter(sens.ChFilterSave("cam/gray/"))                     # SAVE grayscale PNGs (scored)
cam.PushFilter(sens.ChFilterImageResize(640, 360))                 # resize for downstream
cam.PushFilter(sens.ChFilterR8Access())                            # host access R8
manager.AddSensor(cam)                                             # add sensor to manager

# review-only recording setup

# sensor diagnostic — SCORED CORE for sensor demos
buf = cam.GetMostRecentRGBA8Buffer()                               # get RGBA8 buffer

while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        manager.Update()                                            # pump sensors every step
        sys.DoStepDynamics(time_step)                              # advance physics
        buf = cam.GetMostRecentRGBA8Buffer()                       # check buffer each step
        if buf.HasData():                                           # guard: skip until first tick
            print('Buffer received. Resolution: {0}x{1}'.format(buf.Width, buf.Height))  # sensor diagnostic
        if sys.GetChTime() >= sim_end:
            break
