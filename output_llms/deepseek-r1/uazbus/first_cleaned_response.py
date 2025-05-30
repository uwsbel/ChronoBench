import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 0, 1), 
                        100, 100)
terrain.Initialize()


uaz = veh.UAZBUS()
uaz.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
uaz.Initialize(system)
uaz.SetChassisFixed(False)
uaz.SetChassisVisualizationType(veh.VisualizationType_MESH)
uaz.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
uaz.SetWheelVisualizationType(veh.VisualizationType_MESH)




driver = veh.ChInteractiveDriverIRR(uaz.GetVehicle())
steering_controller = veh.ChSteeringController()
driver.SetSteeringController(steering_controller)


driver.SetThrottleGain(0.4)
driver.SetSteeringGain(0.3)
driver.SetBrakingGain(0.3)


pid = veh.ChSpeedController()
uaz.GetVehicle().SetSpeedController(pid)
pid.SetGains(0.4, 0, 0)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('UAZBUS Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-5, 0, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddLightWithShadow(chrono.ChVectorD(-5, 0, 5), chrono.ChVectorD(0, 0, 0), 50, 5, 50, 35, 512, chrono.ChColor(1, 1, 1))
vis.EnableShadows()


step_size = 0.005
realtime_step = 0.01
time_total = 0


uaz.GetVehicle().AttachVisualSystem(vis)


while vis.Run():
    time_current = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    uaz.GetVehicle().Synchronize(time_current, driver_inputs, terrain)
    terrain.Synchronize(time_current)
    
    
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    time.sleep(max(realtime_step - system.GetChTime() + time_total, 0))
    time_total = system.GetChTime()

print("Simulation completed.")