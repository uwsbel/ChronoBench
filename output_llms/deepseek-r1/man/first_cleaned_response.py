import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverMaxIterations(50)





veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")


man = veh.MAN_5t_Vehicle(system)
man.SetContactMethod(chrono.ChContactMethod_NSC)
man.SetChassisCollision(True)
man.SetChassisFixed(False)
man.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.0), chrono.QUNIT))
man.SetTireType(veh.TireModelType_TMEASY)
man.Initialize()

man.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
man.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
man.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
man.SetWheelVisualizationType(veh.VisualizationType_MESH)
man.SetTireVisualizationType(veh.VisualizationType_MESH)




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetDampingF(0.2)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 
                        100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
terrain.Initialize()




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("MAN Truck Simulation")
vis.SetWindowSize(1280, 720)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 6, 2), chrono.ChVectorD(0, 0, 1))
vis.AddTypicalLights()




driver = veh.ChInteractiveDriverIRR(vis)
steering_controller = veh.ChSteeringControllerSMC()
driver.SetSteeringController(steering_controller)
driver.Initialize()




step_size = 0.005
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    man.DriverInputs(driver_inputs)
    
    
    man.Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    system.DoStepDynamics(step_size)
    
    realtime_timer.Spin(step_size)