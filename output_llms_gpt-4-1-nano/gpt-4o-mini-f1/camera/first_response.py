import pychrono as chrono
import pychrono.sensor as sens
import math

# Create the Chrono physical system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create a mesh body and add it to the system
mesh = chrono.ChBodyEasyMesh()
mesh.SetMeshFilename(chrono.GetChronoDataFile("vehichles/meshes/triangulated_meshes/triangulated_mesh.obj"))
mesh.SetPos(chrono.ChVector3d(0, 0, 0))
mesh.SetFixed(True)
mesh.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(mesh)

# Create a sensor manager
manager = sens.ChSensorManager(sys)

# Create a camera sensor and add it to the manager
cam = sens.ChCameraSensor(
    mesh,
    30,  # Update rate in Hz
    chrono.ChFrameD(chrono.ChVector3d(0, 0, 2), chrono.QUNIT),
    1280,  # Image width
    720,   # Image height
    60,    # Field of view
    0.01,  # Near plane
    1000,  # Far plane
    sens.CameraProjectionType_PERSPECTIVE
)
cam.SetName("Camera Sensor")
cam.SetLag(0.0)
cam.SetCollectionWindow(1.0 / 30)  # Time to collect a frame
cam.SetWriteImage(True, "output/images/", "cam_%.6d.png")
cam.SetImageType(sens.ChCameraSensor.ImageType_RGB)
manager.AddSensor(cam)

# Add noise filters to the camera
if False:  # Set to True to enable noise filters
    cam.PushFilter(sens.ChFilterNoise(0.01, 0.01, 0.0))
    cam.PushFilter(sens.ChFilterLensDistortion(0.1, 0.1, 0.0))
    cam.PushFilter(sens.ChFilterRGB())
    cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera RGB"))

# Add a depth filter to the camera
if False:  # Set to True to enable depth filter
    cam.PushFilter(sens.ChFilterDepth())
    cam.PushFilter(sens.ChFilterVisualizeDepth(640, 480, "Camera Depth"))

# Add a buffer filter to the camera
if True:  # Set to True to enable buffer filter
    cam.PushFilter(sens.ChFilterBuffer(1))

# Add a buffer visualization filter to the camera
if True:  # Set to True to enable buffer visualization filter
    cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera Buffer"))

# Initialize the sensor manager
manager.InitializeSensors()

# Set the initial camera orbit parameters
orbit_radius = 5
orbit_speed = 0.1
orbit_angle = 0

# Simulation loop
while True:
    # Update the camera's orbit position
    orbit_angle += orbit_speed
    cam.SetPosOffset(chrono.ChVector3d(orbit_radius * math.cos(orbit_angle), orbit_radius * math.sin(orbit_angle), 2))
    cam.SetLookAtOffset(chrono.ChVector3d(0, 0, 0))

    # Print the camera buffer data
    if cam.HasBuffers():
        print("Camera buffer data:")
        print(cam.GetMostRecentBuffer().GetRGBA8Data())
    
    # Update the sensor manager
    manager.Update()

    # Advance the simulation
    sys.DoStepDynamics(1e-3)