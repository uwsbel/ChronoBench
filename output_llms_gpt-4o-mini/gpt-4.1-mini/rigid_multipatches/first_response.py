import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------

chrono.SetChronoDataPath(chrono.GetChronoDataPath())  # ensure the data path is set

# Create the physical system
system = chrono.ChSystemNSC()

# Enable contact visualization for debugging (optional)
# system.SetContactVisualization(True)
# system.SetContactForceVisualization(True)

# -----------------------------------------------------------------------------
# 2. Create the vehicle
# -----------------------------------------------------------------------------

# Vehicle initial position and orientation
init_loc = chrono.ChVectorD(0, 0, 1.0)  # start 1m above ground
init_rot = chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))

# Create HMMWV vehicle system
vehicle = veh.HMMWV_Full()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_NONE)  # no chassis collision for now
vehicle.Initialize(system, init_loc, init_rot)
vehicle.SetVisualizationType(veh.VisualizationType_MESH)
vehicle.GetVehicle().SetDriveType(veh.DrivelineTypeWV::AWD)  # all wheel drive
vehicle.GetVehicle().SetEngineType(veh.EngineModelType_SHAFTS)

# Enable mesh visualization on all vehicle parts
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

# -----------------------------------------------------------------------------
# 3. Construct the terrain composed of multiple patches with diverse surfaces
# -----------------------------------------------------------------------------

terrain = veh.RigidTerrain(vehicle.GetSystem())

# Patch 1: Flat patch with dirt texture
patch1 = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(7.5, 0, 0), chrono.QUNIT),
                          chrono.ChVectorD(15, 30, 0.1))
patch1.SetColor(chrono.ChColor(0.8, 0.7, 0.5))  # light brown / dirt color
patch1.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 16, 16)
patch1.SetFriction(0.9f)

# Patch 2: Flat patch with asphalt texture, adjacent to patch1
patch2 = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(-7.5, 0, 0), chrono.QUNIT),
                          chrono.ChVectorD(15, 30, 0.1))
patch2.SetColor(chrono.ChColor(0.2, 0.2, 0.2))  # dark asphalt
patch2.SetTexture(veh.GetDataFile("terrain/textures/asphalt.jpg"), 16, 16)
patch2.SetFriction(0.9)

# Patch 3: Mesh patch for bump - create from mesh
mesh_file = veh.GetDataFile("terrain/mesh/bump.obj")
patch3_center = chrono.ChVectorD(0, 15, 0)
patch3_size = chrono.ChVectorD(15, 15, 1.0)
patch3 = terrain.AddPatch(mesh_file, chrono.ChCoordsysD(patch3_center, chrono.QUNIT),
                          patch3_size)
patch3.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
patch3.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 16, 16)
patch3.SetFriction(0.9)

# Patch 4: Heightmap-based patch for terrain elevation variation
# Use sample heightmap and scale appropriately
hm_filename = veh.GetDataFile("terrain/height_maps/hmap16.png")
hm_center = chrono.ChVectorD(0, -15, 0)
hm_size = chrono.ChVectorD(30, 30, 2.0)  # width, length, height
patch4 = terrain.AddPatchHeightMap(hm_filename, chrono.ChVector2D(0, 0), hm_size, 0.07, 200, 200,
                                  chrono.ChCoordsysD(hm_center, chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))
patch4.SetColor(chrono.ChColor(0.4, 0.6, 0.3))
patch4.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64, 64)
patch4.SetFriction(0.8)

terrain.Initialize()

# -----------------------------------------------------------------------------
# 4. Create the interactive driver system
# -----------------------------------------------------------------------------

driver = veh.ChIrrGuiDriver(vehicle)

# Set steering, throttle, brake bias and time response (filtering)
driver.SetSteeringDelta(0.1)  # radians per timestep increment
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.2)
driver.Initialize()

# -----------------------------------------------------------------------------
# 5. Create Irrlicht visualization and attach to the physical system
# -----------------------------------------------------------------------------

application = veh.ChWheeledVehicleVisualSystemIrrlicht()
application.SetWindowTitle("HMMWV on Complex Rigid Terrain")
application.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
application.Initialize()
application.AddTypicalLights()
application.AddSkyBox()
application.AttachSystem(system)

# -----------------------------------------------------------------------------
# 6. Simulation loop parameters
# -----------------------------------------------------------------------------

timestep = 1.0 / 60  # 60 FPS simulation step

# -----------------------------------------------------------------------------
# 7. Simulation loop
# -----------------------------------------------------------------------------

while application.Run():
    # Synchronize modules
    driver_inputs = driver.GetInputs()
    time = system.GetChTime()

    driver.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    application.Synchronize(driver_inputs)

    # Advance simulation by a timestep
    driver.Advance(timestep)
    vehicle.Advance(timestep)
    terrain.Advance(timestep)
    system.DoStepDynamics(timestep)
    application.Advance(timestep)

    # Render scene
    application.Render()