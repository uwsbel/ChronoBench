import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    
    ground_mat = chrono.ChContactMaterialNSC()
    ground = chrono.ChBodyEasyBox(20,20,1,1000,True,True,ground_mat)
    ground.SetPos(chrono.ChVector3d(0,0,-1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
    system.Add(ground)

    
    pend_mat = chrono.ChContactMaterialNSC()
    pend_mat.SetFriction(0.5)
    pend = chrono.ChBodyEasyBox(0.3,0.3,1,1000,True,True,pend_mat)
    pend.SetPos(chrono.ChVector3d(0,0,5))
    pend.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0)))
    pend.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
    system.Add(pend)

    
    joint = chrono.ChLinkLockRevolute()
    joint.SetName('PendulumJoint')
    joint.Initialize(pend,ground,chrono.ChFramed(chrono.ChVector3d(0,0,5),chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0))))
    system.Add(joint)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Sensors')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3,3,3),chrono.ChVector3d(0,0,0))
    vis.AddTypicalLights()

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(100,100,100),chrono.ChColor(1,1,1),1000.0)
    manager.scene.AddPointLight(chrono.ChVector3d(-100,-100,100),chrono.ChColor(1,1,1),1000.0)

    intensity = 1.0
    manager.scene.AddAreaLight(chrono.ChVector3d(0,0,4),chrono.ChColor(intensity,intensity,intensity),500,1)

    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8,0,1),chrono.QuatFromAngleAxis(0,chrono.ChVector3d(0,1,0)))
    cam = sens.ChCameraSensor(
        pend,
        update_rate,
        offset_pose,
        image_width,
        image_height,
        fov
    )
    cam.SetName('Camera Sensor')
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    
    if noise_model == 'CONST_NORMAL':
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0,0.01))
    elif noise_model == 'PIXEL_DEPENDENT':
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02,0.03))
    elif noise_model == 'NONE':
        
        pass

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width,image_height,'RGB Image'))

    
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + '/rgb/'))

    manager.AddSensor(cam)

    
    orbit_radius = 10
    orbit_rate = 1
    ch_time = 0.0
    render_time = 0

    
    
    
    cam_update_rate = update_rate

    t1 = time.time()

    while vis.Run():
        render_time += 1/60

        
        cam_update_rate = (cam_update_rate + 0.1)
        cam.SetUpdateRate(max(1.0, cam_update_rate))

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(step_size)

        ch_time += step_size

        if (ch_time > t_end):
            break

    print("Sim time:", t_end, "Wall time:", time.time()-t1)







save = False

out_dir = 'SENSOR_OUTPUT/'

vis = True





noise_model = 'NONE'


update_rate = 5

lag = 0

exposure_time = 0


image_width = 1280
image_height = 720

fov = 1.408


step_size = 1e-3

t_end = 1000

main()