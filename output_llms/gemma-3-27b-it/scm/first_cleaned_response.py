import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = chrono.ChTerrain()
terrain.Create(system, "SCM", 100, 100, 10, 10)  
terrain.Set_Material_Density(1600)
terrain.Set_Poisson_Ratio(0.3)
terrain.Set_Young_Modulus(8.0e7)
terrain.Set_Cohesion(2.0e3)
terrain.Set_Friction(0.8)


terrain.Set_Visualize_Sinkage(True)


hmmwv = veh.HMMWV(system)


initial_x = 0.0
initial_y = 0.0
initial_z = 0.5
initial_yaw = 0.0  

hmmwv.Set_Pos(chrono.ChVectorD(initial_x, initial_y, initial_z))
hmmwv.Set_Yaw(initial_yaw)


for i in range(hmmwv.Get_Num_Parts()):
    part = hmmwv.Get_Part(i)
    if part:
        part.Set_Visualize(True)
        part.Set_Collide(True)


for wheel in hmmwv.Get_Wheels():
    tire = wheel.Get_Tire()
    tire.Set_Model(veh.TireModelType.RIGID)


moving_patch = chrono.ChMovingPatch()
moving_patch.Set_Terrain(terrain)
moving_patch.Set_Patch_Size(2.0, 2.0, 0.2)  
moving_patch.Set_Patch_Position(chrono.ChVectorD(initial_x, initial_y, initial_z - 0.1))
moving_patch.Set_Update_Rate(chrono.ChTime(0.01)) 
moving_patch.Set_Visualize(True)


driver = veh.HMMWVDriver(hmmwv)
driver.Set_Throttle_Level(0.0)
driver.Set_Steering_Level(0.0)
driver.Set_Braking_Level(0.0)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -15))
vis.AddTypicalLights()


time_step = chrono.ChTime(1.0 / 50.0)  
simulation_time = 0.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().Get_Keyboard().GetKeyDown(chrono.KEY_UP):
        driver.Set_Throttle_Level(1.0)
    elif vis.GetSystem().Get_Keyboard().GetKeyDown(chrono.KEY_DOWN):
        driver.Set_Braking_Level(1.0)
    else:
        driver.Set_Throttle_Level(0.0)
        driver.Set_Braking_Level(0.0)

    if vis.GetSystem().Get_Keyboard().GetKeyDown(chrono.KEY_LEFT):
        driver.Set_Steering_Level(-1.0)
    elif vis.GetSystem().Get_Keyboard().GetKeyDown(chrono.KEY_RIGHT):
        driver.Set_Steering_Level(1.0)
    else:
        driver.Set_Steering_Level(0.0)

    
    chassis = hmmwv.Get_Chassis()
    moving_patch.Set_Patch_Position(chassis.Get_Pos())

    system.DoStepDynamics(time_step)
    simulation_time += time_step