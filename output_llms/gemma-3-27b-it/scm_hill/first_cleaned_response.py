import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.ChTerrain()
terrain.SetTerrainType(veh.ChTerrain.Type_SCM)
terrain.Initialize(system, 200, 200, 10, 10)  


terrain.Set_SCM_max_height(5)
terrain.Set_SCM_min_height(-2)
terrain.Set_SCM_damping(0.2)
terrain.Set_SCM_elasticity(0.05)
terrain.Set_SCM_friction(0.8)


try:
    terrain.LoadHeightMap("heightmap.png")
except:
    print("Heightmap file not found. Using a flat terrain.")
    
    for i in range(terrain.GetXSegments()):
        for j in range(terrain.GetYSegments()):
            terrain.SetHeight(i, j, 0.0)



hmmwv = veh.ChHMMWV()
hmmwv.SetVehicleModel(veh.ChHMMWV.Model.WV)  
hmmwv.Initialize(system)
hmmwv.SetChassisHeight(0.5)  
hmmwv.SetEngineType(veh.ChHMMWV.EngineType.GASOLINE)
hmmwv.SetTireType(veh.ChHMMWV.TireType.OFFROAD)


hmmwv.SetPos(chrono.ChVectorD(0, 2, 0))
hmmwv.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))


driver = veh.ChDriver()
driver.SetInput(veh.ChDriver.InputType.JOYSTICK)  
driver.SetVehicle(hmmwv)
system.Add(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -15))
vis.AddTypicalLights()


timestep = 0.005
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(timestep)
    driver.Synchronize(timestep)
    time += timestep

    if time > 100:
        break 

print("Simulation finished.")