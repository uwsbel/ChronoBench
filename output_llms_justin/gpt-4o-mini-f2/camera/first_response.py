import pychrono as ch
import pychrono.sensor as sens
import math

# Create the Chrono physical system
sys = ch.ChSystemNSC()

# Create a mesh visualization and add it to the system as a fixed body
mesh = ch.ChBodyEasyMesh()
mesh.SetStatic(True)
mesh.SetPos(ch.ChVector3d(0, 0, 0))
mesh.GetVisualShape(0).SetTexture(ch.GetChronoDataFile("textures/blue.png"))
mesh.GetVisualShape(0).SetTriMesh(ch.GetChronoDataFile("meshes/tri_mesh.obj"))
sys.Add(mesh)

# Create a sensor manager to handle the camera sensor
manager = sens.ChSensorManager(sys)

# Create a point light source and add it to the manager
light = sens.ChPointLight()
light.SetPos(ch.ChVector3f(2, 2.5, 10))
light.SetIntensity(1.0)
manager.scene.AddPointLight(light)

# Create a camera sensor and configure its properties
offset_pose = ch.ChFramed(ch.ChVector3d(0, -5, 1), ch.QuatFromAngleAxis(.1, ch.ChVector3d(0, 0, 1)))
cam = sens.ChCameraSensor(mesh, 30, 1280, 720, offset_pose)
cam.SetName("camera sensor")
cam.SetUpdateRate(30)
cam.SetFOV(math.pi / 3)
cam.SetResolution(1280, 720)
cam.SetVisWidth(1280)
cam.SetVisHeight(720)
cam.SetLag(0)

# Add noise models to the camera sensor
cam.AddNoiseFilter(sens.ChNoiseFilterAccess())
cam.AddNoiseFilter(sens.ChNoiseFilterRGB(sens.ChNoiseModelPix(), 0.01, 0.01))
cam.AddNoiseFilter(sens.ChNoiseFilterRGB(sens.ChNoiseModelUniform(), 0.01, 0.01))
cam.AddNoiseFilter(sens.ChNoiseFilterGrayscale(sens.ChNoiseModelUniform(), 0.01, 0.01))
cam.AddNoiseFilter(sens.ChNoiseFilterRGB(sens.ChNoiseModelNormal(), 0.01, 0.01))
cam.AddNoiseFilter(sens.ChNoiseFilterGrayscale(sens.ChNoiseModelNormal(), 0.01, 0.01))

# Add visualization filters to the camera sensor
cam.AddVisualFilter(sens.ChVisualFilterGrayscale())
cam.AddVisualFilter(sens.ChVisualFilterRGB())

# Add the camera sensor to the manager
manager.AddSensor(cam)

# Set the orbit parameters for the camera sensor
orbit_radius = 5
orbit_rate = 0.1
offset_pose = ch.ChFramed(ch.ChVector3d(0, -orbit_radius, 1), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 0, 1)))
cam.SetOffsetPose(offset_pose)

# Simulation loop
step_number = 0
while (sys.GetChTime() < 10):
    # Update the manager (processes all sensors)
    manager.Update()
    
    # Dynamically update the camera's offset pose to create an orbit effect
    cam.SetOffsetPose(ch.ChFramed(
        ch.ChVector3d(0, -orbit_radius * math.cos(orbit_rate * sys.GetChTime()), 1),
        ch.QuatFromAngleAxis(orbit_rate * sys.GetChTime(), ch.ChVector3d(0, 0, 1))
    ))
    
    # Print out the camera buffer data
    rgb_buffer = cam.GetMostRecentRGBBuffer()
    gray_buffer = cam.GetMostRecentGrayscaleBuffer()
    print("RGB buffer: ", rgb_buffer)
    print("Grayscale buffer: ", gray_buffer)
    
    # Advance the simulation by one step
    sys.DoStepDynamics(1e-3)
    step_number += 1