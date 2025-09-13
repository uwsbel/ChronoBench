import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

# Set data paths for Chrono and Vehicle assets
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Simulation parameters
step_size = 0.005
render_step = 1.0 / 50  # 50 Hz rendering

# 1. Initialize system and collision settings
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# 2. Create HMMWV vehicle
vehicle = veh.HMMWV_Full(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
vehicle.Initialize()
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

# 3. Create SCM deformable terrain
terrain = veh.SCMDeformableTerrain(system)
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
terrain.Initialize(20.0, 10.0, 0.04)  # 20m x 10m terrain patch

# Soil parameters (moist clay)
p = {
    "mu": 0.8,
    "Kc": 0.2e4,
    "Kphi": 0.5e6,
    "n": 1.1,
    "cohesion": 0.2e4,
    "shear_angle": 20,
    "Kelastic": 2e6,
    "damping": 0.2e4
}
terrain.SetSoilParameters(p["mu"], p["Kc"], p["Kphi"], p["n"], p["cohesion"], 
                          p["shear_angle"], p["Kelastic"], p["damping"])

# 4. Create driver system
driver = veh.ChInteractiveDriverIRR(vehicle.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()

# 5. Create Irrlicht visualization
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Deformable Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddLightWithShadow(chrono.ChVectorD(5, 5, 5), chrono.ChVectorD(0, 0, 0), 10, 2, 10, 40, 512)
vis.AddTypicalLights()

# 6. Simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
while vis.Run():
    time = system.GetChTime()
    
    # Render scene
    if time % render_step < step_size:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    # Get driver inputs
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Update vehicle and terrain
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    # Advance simulation
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    # Maintain real-time execution
    realtime_timer.Spin(step_size)

print("Simulation completed")