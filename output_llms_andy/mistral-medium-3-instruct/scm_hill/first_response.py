import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.terrain as terrain
import pychrono.irrlicht as chronoirr
import numpy as np

# =============================================================================
# Initialize the PyChrono environment
# =============================================================================
chrono.SetChronoOutputMode(chrono.ChOutputMode.CONSOLE)

# Create the physical system and set solver parameters
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
system.SetSolverType(chrono.ChSolver.Type_PSSOR)
system.SetMaxItersSolverSpeed(40)
system.SetMaxItersSolverStab(40)
system.SetSolverWarmStarting(True)

# =============================================================================
# Create the HMMWV vehicle
# =============================================================================
# Vehicle parameters
vehicle_params = veh.HMMWV_VehicleParameters()
vehicle_params.chassis_fixed = False
vehicle_params.chassis_pos = chrono.ChVectorD(0, 0.5, 0)

# Create the vehicle
vehicle = veh.HMMWV(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_vehicle.json"), vehicle_params)
vehicle.Initialize(chrono.GetChronoDataPath() + "vehicle/hmmwv/")
vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoorD(0, 0.5, 0))
vehicle.SetInitFwdVel(5)

# =============================================================================
# Create the SCM deformable terrain
# =============================================================================
# Terrain parameters
terrain_params = terrain.SCMDeformableTerrainParameters()
terrain_params.soil_parameters = terrain.SCMDeformableSoilParameters()
terrain_params.soil_parameters.bevam_gamma = 0.8
terrain_params.soil_parameters.bevam_beta = 0.2
terrain_params.soil_parameters.K_phi = 1.0e5
terrain_params.soil_parameters.K_rho = 1.0e5
terrain_params.soil_parameters.K_plasticity = 1.0e5
terrain_params.soil_parameters.mu = 0.5
terrain_params.soil_parameters.cohesion = 0

# Create height map (flat terrain with small bump)
sizeX = 20
sizeY = 20
nx = 100
ny = 100
height_map = np.zeros((nx, ny), dtype=float)
for i in range(nx):
    for j in range(ny):
        x = (i - nx/2) * sizeX / nx
        y = (j - ny/2) * sizeY / ny
        height_map[i, j] = 0.1 * np.exp(-(x*x + y*y)/10)

# Create the terrain
terrain = terrain.SCMDeformableTerrain(system)
terrain.Initialize(terrain_params, height_map, sizeX, sizeY, nx, ny)
terrain.SetPlotType(terrain.SCMDeformableTerrain.PlotType.SURFACE)

# =============================================================================
# Create the driver system
# =============================================================================
driver = veh.ChDriverHMMWV(system)
driver.Initialize(chrono.GetChronoDataFile("vehicle/hmmwv/HMMWV_Driver.json"))
driver.SetSteeringController(veh.ChPathFollowerDriver(vehicle, 5.0))
driver.SetThrottleController(veh.ChPathFollowerDriver(vehicle, 5.0))

# =============================================================================
# Create the visualization system
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(5, 2, 0), chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10)
vis.AddLight(chrono.ChVectorD(-5, 5, 5), chrono.ChVectorD(0, 0, 0), 10)
vis.SetSymbolScale(0.1)

# =============================================================================
# Simulation loop
# =============================================================================
# Simulation parameters
step_size = 1e-3
end_time = 10.0
current_time = 0.0

# Main simulation loop
while vis.Run() and current_time < end_time:
    # Update time
    current_time = system.GetChTime()

    # Update driver inputs
    driver.Synchronize(current_time)

    # Update vehicle
    vehicle.Synchronize(current_time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())

    # Update terrain
    terrain.Synchronize(current_time)

    # Advance simulation
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(step_size)

    # Print simulation time
    if int(current_time * 10) % 10 == 0:
        print("Simulation time:", current_time)