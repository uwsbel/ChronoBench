import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Update with your Chrono data path

# Create the vehicle system
vehicle_system = veh.VehicleSystem()
vehicle_system.SetChTimeStep(1e-3)  # 1ms time step

# Create the terrain
terrain = veh.RigidTerrain(vehicle_system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.1)
terrain.SetContactMaterialProperties(2e7, 0.3)

# Create a flat patch of terrain
patch = terrain.AddPatch(chrono.ChCoordinatesysD(chrono.VECT_X, chrono.QUNIT),
                         100, 100,  # dimensions
                         0, 0,      # grid spacing
                         True)      # visualize

# 2. Add the Gator vehicle
gator = veh.Gator(vehicle_system)
gator.SetContactFrictionCoefficient(0.9)
gator.SetContactRestitutionCoefficient(0.1)
gator.SetContactMaterialProperties(2e7, 0.3)

# Set initial vehicle location
gator.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
gator.SetInitFwdVel(5)  # m/s

# Initialize the vehicle
gator.Initialize()

# 3. Set visualization types for different vehicle parts
gator.GetChassisBody().SetVisualizationType(irr.VisualizationType_MESH)
gator.GetChassisBody().GetVisualModel().SetMeshFile(veh.GetDataFile("gator/chassis.obj"))
gator.GetChassisBody().GetVisualModel().SetTextureFile(veh.GetDataFile("gator/chassis.png"))

for i in range(gator.GetNumberWheels()):
    wheel = gator.GetWheelBody(i)
    wheel.SetVisualizationType(irr.VisualizationType_MESH)
    wheel.GetVisualModel().SetMeshFile(veh.GetDataFile("gator/wheel.obj"))
    wheel.GetVisualModel().SetTextureFile(veh.GetDataFile("gator/wheel.png"))

# 4. Add interactive driver system
driver = veh.ChPathFollowerDriver(gator, "my_path", "path.plt")  # Assuming you have a path file
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetTargetSpeed(5)
driver.Initialize()

# 5. Set up sensor manager with point lights and camera
sensor_manager = veh.ChSensorManager(vehicle_system)
sensor_manager.SetSynchronizationMode(chrono.ChSensorManager.SyncMode_FIXED_STEP)

# Add point lights
light1 = chrono.ChPointPointLight()
light1.SetLightPos(chrono.ChVectorD(1, 1, 5))
light1.SetLightDir(chrono.ChVectorD(-1, -1, -5))
sensor_manager.AddLight(light1)

light2 = chrono.ChPointPointLight()
light2.SetLightPos(chrono.ChVectorD(-1, 1, 5))
light2.SetLightDir(chrono.ChVectorD(1, -1, -5))
sensor_manager.AddLight(light2)

# Add camera attached to vehicle chassis
camera = chrono.ChCameraSensor(gator.GetChassisBody(),  # parent body
                              10,                       # update rate (Hz)
                              chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 1.5),  # position
                                             chrono.Q_from_AngAxis(chrono.PI/4, chrono.VECT_Z)),  # orientation
                              640, 480)                # image dimensions
camera.SetName("main_camera")
camera.SetLensModel(chrono.ChLensModelType::PINHOLE)
camera.SetFocalLength(0.05)  # 50mm focal length
sensor_manager.AddSensor(camera)

# 6. Create visualization system
vis = veh.ChVehicleVisualSystemIrrlicht()
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVectorD(0, 0, 1.75), 6.0, 0.5)
vis.Initialize()
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AddLogo()
vis.AttachVehicle(gator.GetChassisBody())

# 7. Simulation loop
time_step = 1e-3
sim_time = 10.0  # simulation duration in seconds

while vehicle_system.GetChTime() < sim_time:
    # Get current time
    time = vehicle_system.GetChTime()

    # Update driver inputs
    driver.Synchronize(time)
    driver.Advance(time_step)

    # Update terrain
    terrain.Synchronize(time)
    terrain.Advance(time_step)

    # Update vehicle
    gator.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    gator.Advance(time_step)

    # Update sensor manager
    sensor_manager.Synchronize(time)
    sensor_manager.Advance(time_step)

    # Update visualization
    vis.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    vis.Advance(time_step)

    # Render camera images
    sensor_manager.Render()

    # Save camera images if needed
    if int(time * 10) % 10 == 0:  # Save every 0.1 seconds
        camera.SaveLastImage("camera_frame_" + str(int(time*10)) + ".png")

    # Render visualization
    vis.Render()

print("Simulation completed successfully.")