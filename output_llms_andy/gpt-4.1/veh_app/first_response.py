import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import time

# 1. Initialize PyChrono
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# 2. Create the vehicle system
initLoc = chrono.ChVectorD(0, 0, 0.5)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.Initialize()

# Set visualization types for different parts
gator.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetWheelVisualizationType(veh.VisualizationType_MESH)
gator.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)

# 3. Add rigid terrain
terrain = veh.RigidTerrain(gator.GetSystem())
terrain_mat = chrono.ChMaterialSurfaceNSC()
patch = terrain.AddPatch(
    terrain_mat,
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(0, 0, 1),
    100, 100
)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# 4. Add interactive driver
driver = veh.ChDriverInteractive(gator)
driver.Initialize()

# 5. Set up sensor manager
manager = sens.ChSensorManager(gator.GetSystem())

# Add point lights
manager.scene.AddPointLight(chrono.ChVectorF(10, 0, 10), chrono.ChColor(1, 1, 1), 500)
manager.scene.AddPointLight(chrono.ChVectorF(-10, 0, 10), chrono.ChColor(1, 1, 1), 500)

# Add a camera to the chassis
cam_offset = chrono.ChVectorF(0.5, 0, 1.2)
cam_rot = chrono.Q_from_AngAxis(math.radians(15), chrono.ChVectorF(0, 1, 0))
camera = sens.ChCameraSensor(
    gator.GetChassisBody(),  # Attach to chassis
    30.0,                    # Update rate [Hz]
    chrono.ChFrameD(cam_offset, cam_rot),  # Offset pose
    1280, 720,               # Image width, height
    math.radians(60)         # FOV
)
camera.SetName("Camera Sensor")
camera.PushFilter(sens.ChFilterRGBA8Access())  # To access image data
manager.AddSensor(camera)

# 6. Simulation loop
step_size = 1e-3
render_step = 0.05  # Render every 0.05s
time_end = 5.0
time_sim = 0
render_time = 0

print("Simulation started. Press W/A/S/D to drive, SPACE to brake, Q to quit.")

while time_sim < time_end:
    # 1. Get driver inputs
    driver.Synchronize(time_sim)
    driver.Advance(step_size)

    # 2. Synchronize modules
    terrain.Synchronize(time_sim)
    gator.Synchronize(time_sim, driver.GetInputs(), terrain)
    
    # 3. Advance modules
    terrain.Advance(step_size)
    gator.Advance(step_size)

    # 4. Update sensor manager (renders sensors)
    manager.Update()

    # 5. Advance simulation time
    time_sim += step_size

    # Optional: Print status or render at intervals
    if time_sim >= render_time:
        print(f"Time: {time_sim:.2f} s")
        render_time += render_step

print("Simulation finished.")