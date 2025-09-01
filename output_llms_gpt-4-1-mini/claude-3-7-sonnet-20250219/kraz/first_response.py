import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
# Initialize PyChrono environment
# ---------------------------------------------------------------------

# Set the path to Chrono data directory
chrono.SetChronoDataPath('./data/')

# Create a ChSystem
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create a rigid terrain
# ---------------------------------------------------------------------
terrain = veh.RigidTerrain(system)
terrain_file = veh.GetDataFile("terrain/rigid_terrain.json")
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(), terrain_file)
terrain.Initialize()

# ---------------------------------------------------------------------
# Create the Kraz vehicle
# ---------------------------------------------------------------------
# Vehicle initial position and orientation
initLoc = chrono.ChVectorD(0, 0.5, 0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Create the Kraz vehicle from JSON specification file
vehicle_file = veh.GetDataFile("vehicle/Kraz_AP1/vehicle/Kraz_AP1_Vehicle.json")
powertrain_file = veh.GetDataFile("vehicle/Kraz_AP1/powertrain/Kraz_AP1_SimpleMapPowertrain.json")
tire_file = veh.GetDataFile("vehicle/Kraz_AP1/tire/Kraz_AP1_TMeasyTire.json")

kraz = veh.WheeledVehicle(system, vehicle_file)
kraz.Initialize(chrono.ChCoordsysD(initLoc, initRot))
kraz.SetChassisVisualizationType(veh.VisualizationType_MESH)
kraz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
kraz.SetWheelVisualizationType(veh.VisualizationType_MESH)

# Create and initialize the powertrain system
powertrain = veh.SimpleMapPowertrain(powertrain_file)
kraz.InitializePowertrain(powertrain)

# Create and initialize the tires
for axle in range(kraz.GetNumberAxles()):
    for side in range(2):
        wheel = kraz.GetWheel(axle, side)
        tire = veh.TMeasyTire(tire_file)
        kraz.InitializeTire(tire, wheel, veh.VisualizationType_MESH)

# ---------------------------------------------------------------------
# Create the driver system
# ---------------------------------------------------------------------
driver_data = veh.DataDriverSTR(veh.GetDataFile("driver/Generic_STR.json"))
driver = veh.ChDataDriverSTR(kraz.GetVehicle())
driver.Initialize()

# ---------------------------------------------------------------------
# Create the visualization system
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("Kraz Vehicle Simulation")
vis.SetWindowSize(1280, 720)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetCameraPosition(chrono.ChVectorD(0, 4, -8), chrono.ChVectorD(0, 0, 0))

vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-10, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.AddGrid(5, 5, 20, 20, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.01, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))

vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))

# ---------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------
step_size = 1e-3  # Step size
real_time_factor = 1.0  # Real-time factor
max_sim_time = 1000  # Maximum simulation time in seconds

# Initialize simulation time and real-time accumulator
sim_time = 0
rt_timer = chrono.ChRealtimeCumulative()

# ---------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------
while vis.Run() and sim_time < max_sim_time:
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    
    # Update modules (vehicle, terrain, etc.)
    driver.Synchronize(sim_time)
    terrain.Synchronize(sim_time)
    kraz.Synchronize(sim_time, driver_inputs, terrain)
    vis.BeginScene()
    vis.Render()
    
    # Camera follows the vehicle
    vehicle_pos = kraz.GetVehicle().GetChassisBody().GetPos()
    vis.SetCameraPosition(vehicle_pos + chrono.ChVectorD(-10, 2, 0), vehicle_pos)
    
    # Advance simulation for one timestep
    system.DoStepDynamics(step_size)
    vis.EndScene()
    
    # Increment simulation time
    sim_time += step_size
    
    # Enforce real-time simulation
    rt_timer.Spin(step_size * real_time_factor)

# Clean up
vis.GetDevice().closeDevice()
print("Simulation completed.")