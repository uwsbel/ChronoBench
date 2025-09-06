import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))


hmmwv = veh.HMMWV_Vehicle("HMMWV", veh.RigidTerrain.VehicleTerrain.SCM)
hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysd(
    chrono.ChVector3d(0, 0, 1.0),  
    chrono.ChQuaterniond(1, 0, 0, 0)  
))
hmmwv.SetTireType(veh.TireModelType_RIGID)  
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


terrain = veh.SCMTerrain(system)
terrain.SetSoilParameters(
    Bekker_Kphi=0.82e6,    
    Bekker_Kc=0.14e4,      
    Bekker_n=1.0,
    Mohr_cohesion=0.017e4, 
    Mohr_friction=30,      
    Janosi_shear=0.01e-2   
)
terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.2)  
terrain.Initialize(20, 40, 0.04)  


terrain.AddMovingPatch(hmmwv.GetChassisBody(), chrono.ChVector3d(0, 0, 0), 5.0, 3.0)


driver = veh.ChInteractiveDriverIRR(hmmwv.GetVehicle())
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on SCM Deformable Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 8, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


hmmwv.GetVehicle().SetVisualSystem(vis)
driver.SetVisualSystem(vis)


step_size = 0.002
frame_interval = 0.02  
time = 0
step = 0
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    driver.Synchronize(time)
    hmmwv.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    
    
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    realtime_timer.Spin(step_size)
    
    
    if step % int(frame_interval / step_size) == 0:
        vis.WriteImageToFile(f"frame_{step:05d}.png")
    
    step += 1