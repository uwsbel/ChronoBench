import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math
import numpy as np






chrono.SetChronoDataPath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data/"))


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(4.0)






vehicle_file = veh.GetDataFile("hmmwv/vehicle/HMMWV_Vehicle.json")
tire_file = veh.GetDataFile("hmmwv/tire/HMMWV_RigidTire.json")
powertrain_file = veh.GetDataFile("hmmwv/powertrain/HMMWV_ShaftsPowertrain.json")


init_pos = chrono.ChVectorD(0, 1.0, 0)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_NONE)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
hmmwv.SetTireType(veh.TireModelType_RIGID)
hmmwv.SetTireStepSize(1e-3)
hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
hmmwv.Initialize()

vehicle = hmmwv.GetVehicle()
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)






SCM_params = chrono.SCMDeformableSoilParameters()
SCM_params.Bekker_Kphi = 0.2e6   
SCM_params.Bekker_Kc = 0         
SCM_params.Bekker_n = 1.1        
SCM_params.Mohr_cohesion = 0     
SCM_params.Mohr_friction = 30    
SCM_params.Janosi_shear = 0.01   
SCM_params.elastic_K = 4e7       
SCM_params.damping_R = 3e4       


terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(SCM_params)


terrain_length = 100.0  
terrain_width = 100.0   
delta = 0.05            


terrain.Initialize(terrain_length, terrain_width, delta)


def height_function(x, y):
    
    bump_x = terrain_length / 2
    bump_y = terrain_width / 2
    dist = math.sqrt((x - bump_x)**2 + (y - bump_y)**2)
    if dist < 5.0:
        return 0.2 * (1 - dist/5.0)**2
    return 0.0


for ix in range(terrain.GetNx()):
    for iy in range(terrain.GetNy()):
        x = terrain.GetMeshX(ix)
        y = terrain.GetMeshY(iy)
        terrain.SetHeight(ix, iy, height_function(x, y))


terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 30000)
terrain.SetMeshWireframe(True)






driver_data = veh.ChDataDriver(vehicle_file)
driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()


driver_inputs = veh.DriverInputs()
driver_inputs.m_throttle = 0.3
driver_inputs.m_steering = 0.0
driver_inputs.m_braking = 0.0






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()






step_size = 1e-3
realtime_timer = chrono.ChRealtimeStepTimer()

print("Simulation started...")

while vis.Run():
    
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    driver.SetInputs(driver_inputs)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(step_size)
    
    
    realtime_timer.Spin(step_size)
    
    
    if time > 20:
        break

print("Simulation ended.")