import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))
system = chrono.ChSystemNSC()
system.SetSolverMaxIterations(150)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)


vehicle = veh.CityBus()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngZ(0)))
vehicle.Initialize()

vehicle.SetChassisVehicleCollide(True)
vehicle.SetTireStepSize(1e-3)
vehicle.SetTireType(veh.TireModelType_TMEASY)


terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(200, 1, 200), 
                        chrono.ChVectorD(0, 0, 0))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 2, -5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerIrregularInput(0.1)
driver.SetSteeringController(steering_controller)
driver.Initialize()


step_size = 0.02
realtime_step = True
frame_rate = 50

while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    vehicle.DriverInputs(driver_inputs)
    
    
    vehicle.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    vis.Synchronize(time, driver_inputs)
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.Advance(step_size)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if realtime_step:
        chrono.ChRealtimeStep(step_size)