import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = chrono.ChTerrain()
terrain.SetTexture(chrono.ChTexture("textures/dirt.png"))
terrain.SetQuadRoughness(0.01)


terrain.LoadQuadMesh(chrono.ChVectorD(0, 0, 0), 100, 100)
terrain.AddCollisionShape(chrono.ChCollisionShape())
terrain.SetCollisionTrimming(True)
system.Add(terrain)


mesh_col = chrono.ChMesh()
mesh_col.LoadFromFile("Highway_col.obj")
mesh_col.SetMass(0)  
mesh_col.SetCollide(True)
system.Add(mesh_col)

mesh_vis = chrono.ChMesh()
mesh_vis.LoadFromFile("Highway_vis.obj")
mesh_vis.SetMass(0)
mesh_vis.SetCollide(False)
system.Add(mesh_vis)



hmmwv = veh.HMMWV()


hmmwv.SetPos(chrono.ChVectorD(0, 1, 0))
hmmwv.SetTransform(chrono.ChQuaternionD(0, 0, 0))


hmmwv.SetContactMethod(chrono.ChContactMethod.SMC)


hmmwv.SetTireModel(veh.TireModelType.TMEASY)


hmmwv.SetVisualize(True)


system.Add(hmmwv)


driver = veh.ChDriver()
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)
driver.SetVehicle(hmmwv)
system.Add(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on Terrain')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -15))
vis.AddTypicalLights()


timestep = 1.0 / 50.0  
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetChTime() < 5:  
        driver.SetSteering(0.5)  
        driver.SetThrottle(1.0)  
    else:
        driver.SetThrottle(0.0) 

    system.DoStepDynamics(timestep)