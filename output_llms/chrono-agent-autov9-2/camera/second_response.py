"""Orbiting RGB camera sensor imaging a static mesh body.

Model: a single fixed rigid body carrying a triangular mesh (an HMMWV chassis
mesh) with collision geometry, sitting on a fixed ground box. An OptiX-rendered
`ChCameraSensor` rides on the mesh body and orbits around it: its offset pose
starts at ChVector3d(-7, 0, 2) relative to the body and rotates about the body's
vertical (Z) axis at a constant 0.1 rad/s, always looking back at the body so the
mesh stays centered in frame. The camera renders 960x480 RGB images and saves
them to disk. An Irrlicht window provides the standard real-time review view.

System type: NSC (ChSystemNSC). The scene is static (no driven motion); only the
camera's offset pose animates, so the bodies stay at rest under gravity resting
on the ground. Expected behavior: the saved camera images sweep a full circle
around the chassis mesh as the simulation advances.
"""

import os
import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Parameters === geometry / physics / camera constants (no bare literals downstream)
time_step = 1e-3                    # s, physics integration step
sim_end = 4.0                       # s, simulated duration over which the camera orbits
render_fps = 50.0                   # Hz, Irrlicht review-frame cadence
update_rate = 1.0 / time_step       # Hz, sensor sampling rate (one per physics step)

image_width = 960                   # px, camera image width
image_height = 480                  # px, camera image height
horizontal_fov = 1.408              # rad, camera horizontal field of view

orbit_rate = 0.1                    # rad/s, camera angular speed about the body Z axis
offset_pose = chrono.ChVector3d(-7, 0, 2)   # base camera offset on the imaged body
orbit_radius = math.hypot(offset_pose.x, offset_pose.y)   # precomputed once: planar orbit radius
orbit_height = offset_pose.z                              # precomputed once: constant eye height
orbit_phase0 = math.atan2(offset_pose.y, offset_pose.x)   # precomputed once: starting orbit angle

ground_size = chrono.ChVector3d(20.0, 20.0, 1.0)          # m, ground box full extents
ground_density = 1000.0                                   # kg/m^3, ground material density
mesh_density = 1000.0                                     # kg/m^3, mesh body density
mesh_z = ground_size.z * 0.5                              # mesh rests on top of the ground box

# === System & gravity === NSC system; collision is present (mesh + ground), so Bullet is required
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # imaged mesh/ground carry collision geometry for OptiX

# === Contact material === shared NSC material for ground and mesh contacts
contact_mat = chrono.ChContactMaterialNSC()
contact_mat.SetFriction(0.8)
contact_mat.SetRestitution(0.0)

# === Bodies === fixed ground box + a fixed mesh body that the camera images
ground = chrono.ChBodyEasyBox(ground_size.x, ground_size.y, ground_size.z,
                              ground_density, True, True, contact_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -ground_size.z * 0.5))   # top face at z = 0
ground.SetFixed(True)
sys.Add(ground)

mesh_body = chrono.ChBodyEasyMesh(
    chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
    mesh_density,
    True,    # compute mass/inertia from the mesh
    True,    # create a visual asset
    True,    # create collision geometry (OptiX renders only collidable bodies)
    contact_mat,
)
mesh_body.SetPos(chrono.ChVector3d(0, 0, mesh_z))
mesh_body.SetFixed(True)
sys.Add(mesh_body)

imaged_pos = mesh_body.GetPos()   # cache: imaged-body center fetched once, reused as orbit pivot

# === Sensor manager & lighting === OptiX scene lights for the camera sensor
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1.0, 1.0, 1.0), 5000.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# === Camera sensor === orbiting RGB camera attached to the mesh body, 960x480, saving frames
camera = sens.ChCameraSensor(
    mesh_body,                                          # rides on the imaged body
    update_rate,                                        # Hz, one sample per physics step
    chrono.ChFramed(offset_pose, chrono.QUNIT),         # initial offset pose (animated each step)
    image_width,
    image_height,
    horizontal_fov,
)
camera.SetName("orbit_camera")
camera.PushFilter(sens.ChFilterVisualize(image_width, image_height))   # live OptiX preview window
camera.PushFilter(sens.ChFilterSave("cam/orbit_camera/"))              # save 960x480 PNG frames to disk
camera.PushFilter(sens.ChFilterRGBA8Access())                          # expose the RGBA8 frame buffer
manager.AddSensor(camera)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Orbiting Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-9, -9, 5), imaged_pos)
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 40, 40, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === advance physics, orbit the camera offset pose, sample the sensor
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once: physics steps per frame
local_up = chrono.ChVector3d(0, 0, 1)                          # precomputed once: world up for look-at



frame = 0
try:
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            t = sys.GetChTime()
            angle = orbit_phase0 + orbit_rate * t   # orbit the camera about the body Z axis
            eye = chrono.ChVector3d(orbit_radius * math.cos(angle),
                                    orbit_radius * math.sin(angle),
                                    orbit_height)
            forward = (chrono.ChVector3d(0, 0, 0) - eye).GetNormalized()   # look from eye toward the body center
            look_at_quat = chrono.QuatFromVec2Vec(chrono.ChVector3d(1, 0, 0), forward)
            camera.SetOffsetPose(chrono.ChFramed(eye, look_at_quat))
            manager.Update()   # pump the sensor every physics step so it sees each pose
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid sensor or render state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
