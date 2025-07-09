# Initialize the PyChrono system with Signorini-Moreau Contact model
sys = chrono.ChSystemSMC()

# Set the data path for Chrono's assets
chrono.SetChronoDataPath("data/")

# Initialize the visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV Simulation")
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()

# Create the vehicle
car = veh.ARTcar()
car.SetContactMethod(chrono.ChContactMethodType.ContactMethod_Default)
car.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)
car.SetTireModel(chrono.ChTireModel.TireModel_TMEASY)  # TMEASY tire model

# Set vehicle properties
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.SetVisualPrimitive(True)  # Use primitive shapes for visualization

# Add visual components
car_body = chrono.ChVisualShapeBox(chrono.ChVector3d(4, 2, 1))
car.AddVisualShape(car_body)
car_wheels = chrono.ChVisualShapeCylinder(chrono.ChVector3d(0.5, 0.5, 0.5), 0.3)
car.AddVisualShape(car_wheels, 4)  # Add wheels
car_engine = chrono.ChVisualShapeBox(chrono.ChVector3d(1, 1, 1))
car.AddVisualShape(car_engine)

# Create the terrain
terrain = veh.RigidTerrain(sys)
terrain_height = 0.1  # Flat terrain height
terrain_width = 50.0  # Terrain width
terrain_length = 50.0  # Terrain length
patch = terrain.AddPatch(
    patch_mat=chrono.ChMaterial(),
    position=chrono.ChCoordsysd(chrono.ChVector3d(0, 0, terrain_height), chrono.QuatFromAngleX(chrono.CH_PI)),
    length=terrain_length,
    width=terrain_width
)
terrain.Initialize()
terrain.SetFriction(0.5)  # Set terrain friction
terrain.SetHeightMap(chrono.ChFunction_Const(terrain_height))

# Set up the driver system
driver = veh.DriverInteractive(sys, "Driver")
driver.EnableKeyboard(True)  # Enable keyboard controls
driver.SetSteeringAxis(chrono.ChAxis.X)
driver.SetAcceleratorAxis(chrono.ChAxis.Y)
driver.SetBrakeAxis(chrono.ChAxis.Z)

# Add the vehicle and terrain to the system
sys.Add(car)
sys.Add(terrain)

# Set simulation parameters
sys.SetGlobalGravity(chrono.ChVector3d(0, -9.81, 0))
sys.SetTimeStep(0.01)  # 50 FPS
sys.SetSimulationFlags(chrono.ChSimulationFlag.SIM_USE_THREAD | chrono.ChSimulationFlag.SIM_VISU_THREAD)

# Set up contact reporting
contact_report = chrono.ChContactContainer()
sys.GetContactContainer().RegisterContactCallback(contact_report)
contact_report.OnReportContact = lambda self, pA, pB, plane_coord, distance, eff_radius, cforce, ctorque, modA, modB: print(f"Contact: {pA} - {pB} - {distance}")

# Start the simulation loop
while vis.Run():
    sys.DoStepDynamics(sys.GetTimeStep())
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Keep the visualization window open
vis.Run()