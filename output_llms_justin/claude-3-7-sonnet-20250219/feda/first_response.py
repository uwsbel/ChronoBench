import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# ---------------------------------------------------------------------
# Initialize the simulation environment

# Set the path to Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data/"))

# Create the system and specify the solver parameters
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver settings
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# ---------------------------------------------------------------------
# Create the terrain

terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Create a flat terrain patch
terrain_file = chrono.GetChronoDataFile("terrain/height_maps/flat.bmp")
terrain_size = chrono.ChVectorD(200, 1, 200)
terrain_center = chrono.ChVectorD(0, 0, 0)

patch = terrain.AddPatch(patch_mat, 
                        terrain_center,
                        chrono.ChVectorD(0, 1, 0),  # normal vector
                        terrain_size.x, 
                        terrain_size.z)

# Set terrain visualization properties
patch.SetTexture(chrono.GetChronoDataFile("textures/grass.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the FEDA vehicle

# Set vehicle file paths
vehicle_file = os.path.join(os.path.dirname(__file__), "data/vehicle/feda.json")
powertrain_file = os.path.join(os.path.dirname(__file__), "data/powertrain/simple_powertrain.json")
tire_file = os.path.join(os.path.dirname(__file__), "data/tire/rigid_tire.json")

# Initial vehicle position and orientation
init_loc = chrono.ChVectorD(0, 0.5, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)  # quaternion (1,0,0,0) means "no rotation"

# Create the vehicle
vehicle = veh.WheeledVehicle(system, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(init_loc, init_rot))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the powertrain
powertrain = veh.SimplePowertrain(powertrain_file)
vehicle.InitializePowertrain(powertrain)

# Create and initialize the tires
for axle in vehicle.GetAxles():
    tireL = veh.RigidTire(tire_file)
    tireR = veh.RigidTire(tire_file)
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

# ---------------------------------------------------------------------
# Create the Irrlicht visualization application

app = veh.ChWheeledVehicleIrrApp(vehicle, "FEDA Vehicle Simulation")

app.SetSkyBox()
app.AddTypicalLights()
app.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
app.SetChaseCamera(chrono.ChVectorD(0, 1.5, -6.0), 6.0, 0.5)
app.SetTimestep(1e-3)

# ---------------------------------------------------------------------
# Create the interactive driver system

driver = veh.ChIrrGuiDriver(app)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# Set the vehicle's driver
vehicle.SetDriver(driver)

# ---------------------------------------------------------------------
# Initialize the simulation and graphics

app.AssetBindAll()
app.AssetUpdateAll()

# Simulation loop parameters
step_size = 1e-3  # 1 ms step size
render_step = 1.0 / 50  # Render at 50 FPS
render_next = 0.0

# ---------------------------------------------------------------------
# Simulation loop

while app.GetDevice().run():
    # Render scene and process user inputs
    app.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1))
    app.DrawAll()
    app.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Advance simulation time
    time = vehicle.GetSystem().GetChTime()
    
    # Advance dynamics by step_size
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    app.Advance(step_size)
    
    # Update the vehicle
    vehicle.Synchronize(time, driver_inputs, terrain)
    app.Synchronize(time, driver_inputs)
    
    # Advance the visualization time only when needed
    if time >= render_next:
        app.Advance(render_step)
        render_next += render_step