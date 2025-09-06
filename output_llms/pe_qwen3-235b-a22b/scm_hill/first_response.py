import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "data"))

# Initialize the system with SMC contact method for deformable terrain
my_system = chrono.ChSystemSMC()

# Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(my_system)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
terrain.SetSoilParameters(
    2e6,    # Bekker Kphi (shear modulus)
    0,      # Bekker Kc (cohesion)
    1e4,    # Bekker n exponent
    0,      # Mohr cohesive limit
    30,     # Mohr friction angle (degrees)
    0.01,   # Janosi shear coefficient
    4e5,    # Elastic stiffness (K)
    2e4     # Damping (C)
)
# Initialize terrain with default flat height map
terrain.Initialize()
terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_PRESSURE, 0, 300)

# Create HMMWV vehicle
vehicle = veh.HMMWV(my_system)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QuatFromAngleZ(0)))
vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType_PRIMITIVES)
vehicle.Initialize()

# Create driver system with constant throttle
driver = veh.ChDriver(vehicle)
driver.SetThrottle(0.5)  # 50% throttle
driver.Initialize()

# Set up Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))  # Camera position
vis.AddTypicalLights()

# Simulation loop parameters
time_step = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()

# Main simulation loop
while vis.Run():
    time = my_system.GetChTime()
    
    # Synchronize components
    driver.Synchronize(time)
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize()
    
    # Advance simulation
    my_system.DoStepDynamics(time_step)
    
    # Update visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    realtime_timer.Spin(time_step)