import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', ''))

# Create the system
sys = chrono.ChSystemSMC()

# Create the SCM terrain
terrain = veh.SCMTerrain(sys)
terrain.SetSoilParameters(
    2e6,    # Bekker Kphi
    0,      # Bekker Kc
    1.1,    # Bekker n exponent
    0,      # Mohr cohesion
    30,     # Mohr friction angle (degrees)
    0.01,   # Damping coefficient
    7000,   # Elastic stiffness (Pa/m)
    0.3     # Friction
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE)
terrain.SetColorMap(chrono.ChColor(0, 0, 1), chrono.ChColor(1, 0, 0))  # Blue to red

# Add a dynamic patch that follows the vehicle
patch = terrain.AddPatch(
    chrono.ChMaterialSurfaceSMC(),
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    20, 20
)
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create the HMMWV vehicle
vehicle = veh.HMMWV_Full(sys)
vehicle.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0.5, 0),  # Initial position
    chrono.QuatFromAngleZ(0)      # Initial orientation
))
vehicle.SetTireType(veh.TireModelType_RIGID)
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.Initialize()

# Create the interactive driver system
driver = veh.ChIrrGuiDriver(vehicle.GetVehicle())
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)

# Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, -5))
vis.AddTypicalLights()

# Simulation loop parameters
step_size = 1e-3  # 0.001 seconds per step
frame_rate = 50
frame_interval = 1.0 / frame_rate

# Main loop
while vis.Run():
    time = sys.GetChTime()
    
    # Synchronize driver and vehicle
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs())
    
    # Advance simulation by one step
    sys.DoStepDynamics(step_size)
    
    # Update terrain patch position to follow chassis
    chassis_pos = vehicle.GetVehicle().GetChassisPos()
    patch.SetPos(chassis_pos)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()