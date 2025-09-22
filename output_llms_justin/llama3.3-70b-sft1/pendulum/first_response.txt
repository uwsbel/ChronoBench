import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.sensor as sens

def main():
    # Create the Chrono system
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # Add a ground body
    ground_mat = chrono.ChContactMaterialNSC()
    ground = chrono.ChBodyEasyBox(20,20,1,1000,True,True,ground_mat)
    ground.SetPos(chrono.ChVector3d(0,0,-1))
    ground.SetFixed(True)
    ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
    system.Add(ground)

    # Add a pendulum
    pend_mat = chrono.ChContactMaterialNSC()
    pend_mat.SetFriction(0.5)
    pend = chrono.ChBodyEasyBox(0.3,0.3,1,1000,True,True,pend_mat)
    pend.SetPos(chrono.ChVector3d(0,0,5))
    pend.SetRot(chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0)))
    pend.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/blue.png'))
    system.Add(pend)

    # Add a revolute joint
    joint = chrono.ChLinkLockRevolute()
    joint.SetName('PendulumJoint')
    joint.Initialize(pend,ground,chrono.ChFramed(chrono.ChVector3d(0,0,5),chrono.QuatFromAngleAxis(.2,chrono.ChVector3d(1,0,0))))
    system.Add(joint)

    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024,768)
    vis.SetWindowTitle('Sensors')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(3,3,3),chrono.ChVector3d(0,0,0))
    vis.AddTypicalLights()

    # Create a sensor manager
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

    # Create a filter graph for post-processing the data from the camera
    if noise_model == 'CONST_NORMAL':
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0,0.01))
    elif noise_model == 'PIXEL_DEPENDENT':
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02,0.03))
    elif noise_model == 'NONE':
        # Don't add any noise models
        pass

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width,image_height,'RGB Image'))

    # Add a filter to save the current image to a png file at a specified interval
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + '/rgb/'))

    manager.AddSensor(cam)

    # Simulation loop
    orbit_radius = 10
    orbit_rate = 1
    ch_time = 0.0
    render_time = 0

    # Update sensor parameters (e.g. update rate) on the fly
    # To do this, we must first get the most recent filter in the chain, which is the sensor itself
    # Then, we can change the update rate and the changes will be propogated to the rest of the system
    cam_update_rate = update_rate

    t1 = time.time()

    while vis.Run():
        render_time += 1/60

        # Update camera sensor parameters
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

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, saves the camera images
save = False
# Directory to save the camera images
out_dir = 'SENSOR_OUTPUT/'
# If true, displays the camera images
vis = True

# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model = 'CONST_NORMAL'
# noise_model = 'PIXEL_DEPENDENT'
noise_model = 'NONE'

# Camera's update rate
update_rate = 5
# Lag (in seconds) between sensing and when data becomes accessible
lag = 0
# Exposure (in seconds) of each image
exposure_time = 0

# Image width and height
image_width = 1280
image_height = 720
# Camera's horizontal field of view
fov = 1.408

# Simulation step size
step_size = 1e-3
# Simulation end time
t_end = 1000

main()