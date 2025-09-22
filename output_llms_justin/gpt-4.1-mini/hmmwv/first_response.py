import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# 1. Initialize the PyChrono environment and core components.

# Set Chrono data path (adjust the path below as needed)
chrono.SetChronoDataPath(chrono.GetChronoDataPath())

# Choose contact method
contact_method = chrono.ChContactMethod_NSC  # Non-smooth contact (default)

# Create the vehicle system: HMMWV
# Specify initial position and orientation
initLoc = chrono.ChVectorD(0, 0, 0.6)  # a bit above ground (z=0.6 m)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Create the vehicle system with specified contact method and tire model TMEASY
hmmwv = veh.HmmwvVehicle(contact_type=contact_method, tire_model=veh.HmmwvVehicle.TireModelType_TMEASY)

# Set initial position and orientation of the vehicle chassis
hmmwv.SetInitPosition(chrono.ChFrameD(initLoc, initRot))

# Use primitive visualization to speed up rendering
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

# Set up the vehicle system
hmmwv.Initialize()

# 2. Add the required physical systems and objects

# Create the rigid terrain
terrain = veh.RigidTerrain(hmmwv.GetSystem())
terrain.SetPlane(0, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngY(0)))  # flat at z=0

# Set rigid terrain surface material (default friction is fine)
patch_mat = chrono.ChMaterialSurfaceSMC()
patch_mat.SetFriction(0.9)
terrain.SetContactSurfaceMaterial(patch_mat)

# Define terrain patch dimensions
patch_length = 150
patch_width = 150

patch = terrain.AddPatch(patch_mat,
                         chrono.ChVectorD(0, 0, 0),
                         chrono.ChVectorD(patch_length, 0, 0),
                         chrono.ChVectorD(0, patch_width, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 25, 25)

terrain.Initialize()

# 3. Set up an interactive driver system to handle steering, throttle, and braking

# Create the Irrlicht visualization application
application = veh.ChWheeledVehicleIrrApp(hmmwv, "HMMWV on Rigid Terrain - TMEASY Tires", irr.dimension2du(1280, 720))

application.SetTimestep(0.02)  # 50 FPS
application.AddTypicalLights()
application.AddLogo()

# Initialize the visualization system
application.Initialize()
application.SetCameraPosition(chrono.ChVectorD(8, -18, 5))
application.SetCameraTarget(hmmwv.GetChassis().GetPos())

# Create the interactive driver
driver = veh.ChWheeledVehicleVisualSystemIrrlicht(hmmwv.GetSystem())
# Alternatively, can use app.GetDriver
driver = application.GetDriver()
driver.SetInputDelay(0.1)  # Add some input delay
driver.Initialize()

# Simulation loop parameters
timestep = 0.02
step_per_render = 1

# Simulation loop
while application.GetDevice().run():
    time = hmmwv.GetSystem().GetChTime()

    # Collect inputs from the interactive driver
    driver_inputs = driver.GetInputs()

    # Submit steering, throttle, and braking inputs to the vehicle
    hmmwv.SteeringSystem().SetInput(driver_inputs.m_steering)
    hmmwv.SetThrottle(driver_inputs.m_throttle)
    hmmwv.SetBrake(driver_inputs.m_braking)

    # Advance simulation by one timestep
    application.BeginScene()
    application.DrawAll()

    hmmwv.Advance(timestep)
    terrain.Advance(timestep)
    driver.Advance(timestep)

    application.EndScene()

    # Sync simulation to real time (runs at ~50 FPS)
    # The irrigation app already calls Synchronize but we can enforce real-time stepping:
    chrono.ChRealtimeStepTimer().Advance(timestep)