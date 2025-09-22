import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

import math

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    
    
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

    
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(mphysicalSystem)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Test')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5,0.1,-0.2), chrono.ChVector3d(0,1,0))
    vis.AddTypicalLights()

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(0, 0, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(10, 0, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(0, 10, 5))
    manager.scene.AddPointLight(chrono.ChVector3f(intensity, intensity, intensity), chrono.ChVector3f(10, 10, 5))

    
    
    
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

        
        cam_pose = chronoirr.CameraPoseFromFrame(cam.GetOffsetPose())

        vis.BindCamera(chronoirr.Camera(cam_pose, chronoirr.Camera::Vector3d(0.0, 0.0, 0.0)))

        
        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        cam.Synchronize(ch_time)
        vis.Synchronize(ch_time)

        
        ch_time += step_size

    return 0






import os
this_dir = os.path.dirname(os.path.abspath(__file__))
chrono.SetChronoDataPath(os.path.join(this_dir, 'data'))


step_size = 1e-3
update_rate = 30 

image_width = 1280
image_height = 720

tend = 100.0


out_dir = os.path.join(this_dir, 'sensor_out')

main()