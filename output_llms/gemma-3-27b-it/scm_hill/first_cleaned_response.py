import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = chrono.ChTerrain()
terrain.Set_Texture(chrono.ChTexture("textures/terrain.png"))  
terrain.Set_Size(100, 100)
terrain.Set_Height_Scale(1.0)
terrain.Set_Max_Height(2.0)
terrain.Set_Min_Height(-1.0)
terrain.Set_Horizontal_friction(0.5)
terrain.Set_Compliance(0.01)


height_map = np.zeros((100, 100))
center_x = 50
center_y = 50
radius = 10
height = 1.0
for i in range(100):
    for j in range(100):
        dist = np.sqrt((i - center_x)**2 + (j - center_y)**2)
        if dist <= radius:
            height_map[i, j] = height * (1 - dist / radius)

terrain.Set_Height_Map(height_map)
system.Add(terrain)


hmmwv = veh.HMMWV()
hmmwv.Set_Vehicle_Mass(2700)
hmmwv.Set_Engine_Torque(500)
hmmwv.Set_Max_Speed(30)
hmmwv.Set_Brake_Torque(1000)
hmmwv.Set_Tire_Rolling_Radius(0.35)
hmmwv.Set_Tire_Width(0.3)
hmmwv.Set_Tire_Pressure(1.5)

hmmwv.Initialize(system)
hmmwv.Set_Pos(chrono.ChVectorD(0, 1.0, 0))


driver = veh.ChSimpleDriver(hmmwv)
driver.Set_Input_Speed(0.0)
driver.Set_Input_Steering(0.0)
driver.Set_Input_Braking(0.0)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()


timestep = 0.002
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver.Set_Input_Speed(10.0)  
    driver.Set_Input_Steering(0.0)
    driver.Set_Input_Braking(0.0)

    
    system.DoStepDynamics(timestep)

    
    driver.Synchronize(timestep)

    time += timestep