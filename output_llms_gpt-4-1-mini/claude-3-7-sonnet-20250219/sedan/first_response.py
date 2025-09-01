import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math

# ---------------------------------------------------------------------
# Global parameters
# ---------------------------------------------------------------------

# Simulation step size
step_size = 1e-3

# Simulation length (in seconds)
simulation_time = 1000

# BMW E90 Sedan JSON specification files
vehicle_file = "vehicle/BMW_E90.json"
powertrain_file = "powertrain/BMW_E90_SimplePowertrain.json"
tire_file = "tire/BMW_E90_TMeasy.json"

# Initial vehicle position and orientation
init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)

# Path to Chrono data directory
chrono_data_dir = "/path/to/chrono/data/"  # Change this to your Chrono data directory

# ---------------------------------------------------------------------
# Initialize the Chrono system
# ---------------------------------------------------------------------

# Create the Chrono system
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# Set solver and integrator parameters
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetSolverMaxIterations(150)
sys.SetMaxPenetrationRecoverySpeed(4.0)
sys.SetMinBounceSpeed(2.0)
sys.SetSolverForceTolerance(0)

# ---------------------------------------------------------------------
# Create the terrain
# ---------------------------------------------------------------------

terrain = veh.RigidTerrain(sys)

# Create terrain patch
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

# Define the terrain dimensions
terrain_length = 200.0  # Length (X direction)
terrain_width = 200.0   # Width (Y direction)

patch = terrain.AddPatch(patch_mat, 
                         chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                         terrain_length, terrain_width)

# Set terrain visualization properties
patch.SetTexture(chrono_data_dir + "terrain/textures/tile4.jpg", 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

# Add BMW logo on the terrain
logo = patch.AddPlot(chrono_data_dir + "vehicle/logo/bmw_logo.png", 
                    chrono.ChVectorD(0, 0, 0.02), 
                    10, 10)

# Initialize the terrain
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the vehicle
# ---------------------------------------------------------------------

# Create the BMW E90 vehicle
vehicle = veh.WheeledVehicle(sys, vehicle_file)
vehicle.Initialize(chrono.ChCoordsysD(init_pos, init_rot))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the powertrain system
powertrain = veh.SimplePowertrain(powertrain_file)
vehicle.InitializePowertrain(powertrain)

# Create and initialize the tires
for axle in vehicle.GetAxles():
    tireL = veh.TMeasyTire(tire_file)
    tireR = veh.TMeasyTire(tire_file)
    
    vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
    vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

# Set collision detection mode for the vehicle
vehicle.SetCollide(veh.CollisionType_ALL)

# ---------------------------------------------------------------------
# Create the interactive driver
# ---------------------------------------------------------------------

driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.1)
driver.Initialize()

# ---------------------------------------------------------------------
# Create the visualization system
# ---------------------------------------------------------------------

vis = irr.ChIrrApp(sys, "BMW E90 Sedan Simulation", irr.dimension2du(1280, 720))
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddLogo(chrono_data_dir + "logo/chronoengine_logo.png")

# Set camera parameters
vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)

# Set visualization attributes
vis.SetSymbolscale(0.5)
vis.SetShowInfos(True)

# Initialize visualization
vis.Initialize()
vis.AddShadowAll()
vis.SetTimestep(step_size)

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------

# Number of steps
time = 0
render_steps = int(0.01/step_size)

while vis.GetDevice().run():
    # Render scene
    if sys.GetChTime() >= time:
        vis.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        vis.DrawAll()
        
        # Output vehicle information
        driver_inputs = driver.GetInputs()
        vehicle_speed = vehicle.GetVehicle().GetVehicleSpeed()
        
        info = "Speed: {:.1f} km/h    Throttle: {:.2f}    Steering: {:.2f}    Braking: {:.2f}".format(
            vehicle_speed * 3.6, driver_inputs.m_throttle, driver_inputs.m_steering, driver_inputs.m_braking)
        vis.GetDevice().getGUIEnvironment().addStaticText(irr.core.stringw(info),
                                                         irr.core.recti(10, 10, 500, 30),
                                                         False)
        
        vis.EndScene()
        time += 0.01
    
    # Get driver inputs based on keyboard
    driver.Synchronize(time)
    
    # Update vehicle
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    terrain.Synchronize(time)
    sys.DoStepDynamics(step_size)
    
    # Update visualization app
    vis.Synchronize("", driver_inputs)
    
    # Check if simulation should end
    if time > simulation_time:
        break

vis.GetDevice().closeDevice()