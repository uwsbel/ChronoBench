import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath("/path/to/chrono/data/" if os.path.exists("/path/to/chrono/data/") else "")
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


hmmwv = vehicle.VehicleSystem("hmmwv", vehicle.ChVehicleSystem.SMC)
hmmwv.SetChassisPosition(chrono.ChVector3d(0, 0, 0.5))
hmmwv.SetChassisVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(vehicle.VisualizationType_MESH)
hmmwv.Initialize()
hmmwv.SetCollideChassis(False)


soil = vehicle.SCMSoilParameters()
soil.SetCohesion(0.0)
soil.SetFriction(0.4)
soil.SetYoungModulus(2e6)
soil.SetPoissonRatio(0.3)
soil.SetDensity(2000)

terrain = vehicle.SCMDeformableTerrain(hmmwv, soil)
terrain.SetTerrainSize(100.0, 100.0)
terrain.SetHeightMap(vehicle.SCMHeightMap.Flat, 0.0)
terrain.Initialize()
terrain.SetVisualizationType(vehicle.VisualizationType_NONE)


system.Add(hmmwv.GetChassisBody())
system.Add(terrain.GetGroundBody())


driver = vehicle.VehicleDriver(hmmwv)
driver.SetThrottle(0.5)
driver.SetBraking(0.0)


app = irr.IrrlichtApplication(system, "HMMWV SCM Simulation", irr.dimension2du(1024, 768))
app.SetCamera(chrono.ChVector3d(0, -6, 1.5), chrono.ChVector3d(0, 0, 0.5))
app.AddSkyBox()
app.AddCameraLight()
app.AddLightWithShadow(chrono.ChVector3d(10, 10, 10), chrono.ChVector3d(0, 0, 0), 3, 25, 100, chrono.ChColor(0.8, 0.8, 0.8))


time_step = 0.005
simulation_time = 0.0
real_time = 0.0

while app.GetDevice().run():
    
    simulation_time = system.GetChTime()
    steering = 0.5 * math.sin(2 * math.pi * simulation_time / 5.0)
    driver.SetSteering(steering)
    
    
    hmmwv.Synchronize(simulation_time, driver)
    terrain.Synchronize(simulation_time)
    app.Synchronize("HMMWV SCM Simulation", driver)
    
    system.DoStepDynamics(time_step)
    hmmwv.Advance(time_step)
    terrain.Advance(time_step)
    app.Advance(time_step)
    
    
    app.BeginScene()
    app.DrawAll()
    app.EndScene()
    
    
    real_time = system.GetChTime()
    if real_time < simulation_time:
        chrono.ChRealtimeStepDamping(time_step)