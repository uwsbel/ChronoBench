import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.vehicle.driver as driver
import pychrono.vehicle.terrain as terrain
import pychrono.visualization as visualization
import math


chrono.SetChronoDataPath('/usr/local/chrono/data/')
veh.SetVehicleDataPath('/usr/local/chrono.vehicle/data/')

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(veh.ChVehicleContactMethod.NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetPowertrainType(veh.ChVehiclePowertrainModelType.SHV)
hmmwv.SetTireType(veh.ChTireModelType.RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType.PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType.PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType.PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType.NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType.NONE)


scm_terrain = terrain.SCMDeformableTerrain(system, hmmwv.GetChassisBody())
soil_parameters = terrain.SCMParameters()
soil_parameters.mu = 0.9
soil_parameters.K = 2e6
soil_parameters.G = 3e6
soil_parameters.phi = 30.0
soil_parameters.cohesion = 0.0
soil_parameters.density = 2000.0
soil_parameters.Kphi = 3.0
soil_parameters.Kvol = 0.0
soil_parameters.damping = 0.0
scm_terrain.SetSoilParameters(soil_parameters)
scm_terrain.SetHeightMap("terrain_height_map.bin")  
scm_terrain.SetBumpAmplitude(0.1)
scm_terrain.SetBumpWavelength(2.0)
scm_terrain.Initialize(hmmwv.GetChassisBody(), 100, 100, 0.02)
scm_terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))


driver_inputs = driver.ChDriverInputs()
driver_system = driver.ChIrrGuiDriver(hmmwv, driver_inputs)
driver_system.SetSteeringDelta(0.05)
driver_system.SetThrottleDelta(0.05)
driver_system.SetBrakingDelta(0.05)


vis = visualization.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono HMMWV SCM Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -6, 1.2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()


step_size = 1e-3
real_time_factor = 1.0
simulation_time = 0.0

while vis.Run():
    time = system.GetChTime()
    dt = min(step_size, real_time_factor * (time - simulation_time))
    
    
    driver_system.Synchronize(time)
    driver_inputs = driver_system.GetInputs()
    
    
    hmmwv.Synchronize(time, driver_inputs, scm_terrain)
    scm_terrain.Synchronize(time, hmmwv.GetChassisBody())
    
    
    system.DoStepDynamics(dt)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    simulation_time = time

    
    chrono.ChRealtimeStep(dt)