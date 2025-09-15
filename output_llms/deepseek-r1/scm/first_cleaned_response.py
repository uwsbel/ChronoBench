import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
system = chrono.ChSystemSMC()


vehicle = veh.HMMWV_Vehicle("HMMWV", 
                           veh.RigidTerrain.VehicleType_SEDAN,
                           rigid_tire=True)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngZ(math.pi/2)))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.SCMDeformableTerrain(system)
terrain.SetSoilParameters(2e6,   
                         0,      
                         1.3,    
                         5e4,    
                         35,     
                         1e7,    
                         3e4     
                        )
terrain.EnableMovingPatch(vehicle.GetChassisBody(), 2.0, 4.0)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.2)
terrain.Initialize(8.0, 16.0, 0.02)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3))
vis.AddTypicalLights()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()


step_size = 0.002
realtime_step = 0.02
frame_rate = 50

while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    vehicle.DriverInputs(driver_inputs)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    system.DoStepDynamics(step_size)
    
    
    chrono.ChRealtimeStep(realtime_step)