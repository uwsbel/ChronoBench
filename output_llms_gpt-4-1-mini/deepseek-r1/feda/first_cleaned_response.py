import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


vehicle = veh.FEDA_Vehicle(False, veh.RigidTerrain.ContactMethod_NSC)
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngZ(0)))
vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("FEDA Vehicle Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


driver = veh.ChInteractiveDriverIRR(vis)
steering_time = 1.0  
driver.SetSteeringDelta(1.0/(50.0 * steering_time))
driver.SetThrottleDelta(0.5/50.0)
driver.SetBrakingDelta(0.5/50.0)
driver.Initialize()


step_size = 0.02
render_steps = math.ceil(1.0 / (50 * step_size))
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    time = system.GetChTime()
    
    
    vehicle.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)


del vis
del system