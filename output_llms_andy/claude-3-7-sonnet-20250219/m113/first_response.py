import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# ---------------------------------------------------------------------
# Initialize the simulation environment

# Create the chrono system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Set solver settings
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)

# ---------------------------------------------------------------------
# Create the M113 vehicle

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))

# Create the M113 vehicle, set visualization type, and initialize
vehicle_file = veh.GetDataFile("M113/vehicle/M113_Vehicle.json")
powertrain_file = veh.GetDataFile("M113/powertrain/M113_SimplePowertrain.json")
tire_file = veh.GetDataFile("M113/track_shoe/M113_TrackShoe.json")

m113 = veh.M113("M113", chrono.ChContactMethod_SMC)
m113.SetChassisVisualizationType(veh.VisualizationType_MESH)
m113.SetTrackShoeVisualizationType(veh.VisualizationType_MESH)
m113.SetSprocketVisualizationType(veh.VisualizationType_MESH)
m113.SetIdlerVisualizationType(veh.VisualizationType_MESH)
m113.SetRoadWheelVisualizationType(veh.VisualizationType_MESH)
m113.SetTrackVisualizationType(veh.TrackVisualizationType_TRACK_SHOES)

# Change collision envelope settings
m113.SetCollisionEnvelope(0.01, 0.01, 0.05, 0.05, 0.05)

# Initialize the vehicle at specific position and orientation
m113.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.9, 0), chrono.Q_from_AngZ(0)))

# Set the powertrain
m113.SetPowertrain(veh.ReadPowertrainJSON(powertrain_file))

# Add the vehicle to the system
m113.GetSystem().AddBody(m113.GetChassisBody())

# ---------------------------------------------------------------------
# Create the terrain

terrain = veh.RigidTerrain(system)
patch_material = chrono.ChMaterialSurfaceSMC()
patch_material.SetFriction(0.8)
patch_material.SetRestitution(0.2)

patch = terrain.AddPatch(patch_material, 
                         chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(0, 0, 0), 
                         100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100, 100)
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the driver system

# Create the driver
driver = veh.ChDataDriver(veh.GetDataFile("M113/driver/Acceleration.txt"))
driver.Initialize()

# ---------------------------------------------------------------------
# Create the visualization

# Create the Irrlicht visualization
vis = veh.ChWheeledVehicleIrrApp(m113.GetVehicle())
vis.SetWindowTitle("M113 Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.Initialize()

# Set camera settings
vis.GetCamera().setPosition(chrono.ChVectorD(0, 3, -6))
vis.GetCamera().setTarget(chrono.ChVectorD(0, 0, 0))

# Add lights
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5.5, -2.5), chrono.ChVectorD(0, 0, 0), 3, 4, 10, 40, 512)

# ---------------------------------------------------------------------
# Simulation loop

real_time_factor = 1.0
step_size = 1e-3
time_end = 20.0

time = 0.0
while vis.GetDevice().run() and time < time_end:
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (process inputs from other modules)
    driver.Synchronize(time)
    terrain.Synchronize(time)
    m113.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)
    
    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    m113.Advance(step_size)
    vis.Advance(step_size)
    
    # Update system
    system.DoStepDynamics(step_size)
    
    # Increment time
    time += step_size
    
    # Enforce real-time
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

# Close Irrlicht application
vis.GetDevice().closeDevice()