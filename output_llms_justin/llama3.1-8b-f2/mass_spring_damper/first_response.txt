import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # ----------------------------------------
    # add a floor, a spring and a mass to it
    # ----------------------------------------
    phys_mat = chrono.ChContactMaterialNSC()
    phys_mat.SetFriction(0.3)
    phys_mat.SetDampingF(0.00000)
    phys_mat.SetCompliance (1e-9)
    phys_mat.SetComplianceT(1e-7)

    floor = chrono.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(chrono.ChVector3d(0,0,-1))
    floor.SetFixed(True)
    mphysicalSystem.Add(floor)

    mass = chrono.ChBody()
    mass.SetMass(50)
    mass.SetInertiaXX(chrono.ChVector3d(5,5,5))
    mass.SetPos(chrono.ChVector3d(0,0,5))
    mphysicalSystem.Add(mass)

    spring = chrono.ChLinkTSDA()
    spring.SetSpringConstant(200000)
    spring.SetDampingConstant(20)
    spring.Initialize(mass, floor, chrono.ChFramed(chrono.ChVector3d(0,0.1,0),chrono.ChQuaterniond(1,0,0,0)))
    mphysicalSystem.Add(spring)

    # -----------------------
    # Create the Irrlicht app
    # -----------------------
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(mphysicalSystem)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Test')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5,0.1,-0.2), chrono.ChVector3d(0,1,0))
    vis.AddTypicalLights()

    # -----------------------
    # Create the sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(0, 0, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(10, 0, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(0, 10, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(10, 10, 5))

    # ----------------------------------------------
    # create a sensor and add it to the sensor manager
    # ----------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 2),
        chrono.ChQuaterniond(1, 0, 0, 0))
    cam = sens.ChCameraSensor(
        mass,
        0.1,
        update_rate,
        offset_pose,
        image_width,
        image_height)
    cam.SetName("Camera Sensor")
    m_filter = sens.ChFilterVisualize(image_width, image_height, out_dir + "/out%04d.png")
    cam.PushFilter(m_filter)
    manager.AddSensor(cam)

    # -----------------------
    # Simulate the system
    # -----------------------
    orbit_radius = 5
    orbit_rate = 1
    ch_time = 0.0

    render_steps = int(step_size / update_rate)
    if render_steps < 1:
        render_steps = 1

    vis.BindCamera(chronoirr.Camera(chrono.CastVector3d(cam.GetOffsetPose().GetTranslation()), chrono.CastVector3d(0, 0, 1)))

    while vis.Run() :
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get the current camera pose
        cam_pose = chronoirr.CameraPoseFromFrame(cam.GetOffsetPose())

        vis.BindCamera(chronoirr.Camera(cam_pose, chronoirr.Camera::Vector3d(0.0, 0.0, 0.0)))

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Synchronize objects (update offsets)
        cam.Synchronize(ch_time)
        vis.Synchronize(ch_time)

        # Increment time
        ch_time += step_size

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
import os
this_dir = os.path.dirname(os.path.abspath(__file__))
chrono.SetChronoDataPath(os.path.join(this_dir, 'data'))

# Simulation step sizes
step_size = 1e-3
update_rate = 30 # in Hz
# Size of the image
image_width = 1280
image_height = 720
# Simulation end time
tend = 100.0

# Directory to save camera images
out_dir = os.path.join(this_dir, 'sensor_out')

main()