import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irrlicht

# Initialize the PyChrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)  # Using NSC contact
vehicle.SetChassisFixed(False)
# Set initial position and orientation (Z-up, at (0,0,1))
init_pos = chrono.ChVectorD(0, 0, 1.0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # Identity quaternion (no rotation)
vehicle.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
vehicle.SetTireType(veh.TireModelType_TMeasy)  # TMEASY tire model
# Enable mesh visualization for all vehicle components
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()

# Create the custom terrain using mesh files
terrain = veh.RigidTerrain(system)
# Add collision mesh patch
patch = terrain.AddPatch(chrono.ChMaterialSurfaceNSC(),
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         "Highway_col.obj",
                         1.0)  # Assuming scale factor is 1.0
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))  # Set a default color
terrain.Initialize()

# Set up Irrlicht visualization
application = irrlicht.ChIrrApp(system, 
                                'PyChrono HMMWV Simulation', 
                                irrlicht.dimension2du(1024, 768),
                                irrlicht.E_DRIVER_TYPE.EDT_OPENGL)
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights(irrlicht.vector3df(30, 100, 30), irrlicht.vector3df(30, 0, 30), 256)
application.AddTypicalCamera(irrlicht.vector3df(0, 3, -6), irrlicht.vector3df(0, 0, 0))

# Load the visual mesh for the terrain in Irrlicht
vis_mesh = application.GetSceneManager().getMesh("Highway_vis.obj")
if vis_mesh:
    vis_node = application.GetSceneManager().addMeshSceneNode(vis_mesh)
    vis_node.setMaterialFlag(irrlicht.EMF_LIGHTING, True)
    # Optionally adjust position if needed
else:
    print("Error: Could not load visual mesh file 'Highway_vis.obj'")

# Bind all visualization assets
application.AssetBindAll()
application.AssetUpdateAll()

# Initialize the interactive driver system
driver = veh.ChInteractiveDriverIRR(application.GetDevice(), vehicle)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)
driver.Initialize()

# Simulation loop parameters
step_size = 1.0 / 50.0  # 50 FPS
realtime_timer = chrono.ChRealtimeStepTimer()

while application.GetDevice().run():
    # Get current simulation time
    time = system.GetChTime()

    # Synchronize modules
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)

    # Advance simulation
    system.DoStepDynamics(step_size)
    realtime_timer.Spin(step_size)  # Enforce real-time step

    # Render the scene
    application.BeginScene()
    application.DrawAll()
    application.EndScene()