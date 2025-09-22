import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh
import numpy as np


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = veh.SCMDeformableTerrain(system)
terrain.SetTerrainSize(200, 200)
terrain.SetQuadResolution(32, 32)
terrain.SetHeightMap(np.random.rand(32, 32) * 0.5)  
terrain.SetSoilProperties(chrono.ChSoilProperties())
terrain.GetSoilProperties().SetCrushability(0.5)
terrain.GetSoilProperties().SetCohesion(1000)
terrain.GetSoilProperties().SetFriction(0.8)
terrain.GetSoilProperties().SetRestitution(0.1)
terrain.Initialize()


hmmwv = veh.HMMWV(system)
hmmwv.SetVehicleMass(1500)
hmmwv.SetEngineType(veh.HMMWV.EngineType.GAS)
hmmwv.SetTireType(veh.HMMWV.TireType.RADIAL)
hmmwv.SetVisualizationType(veh.HMMWV.VisualizationType.SIMPLE)
hmmwv.Initialize()


hmmwv.SetPos(chrono.ChVectorD(0, 1, 0))
hmmwv.SetForward(chrono.ChVectorD(1, 0, 0))


driver = veh.SimpleDriver(hmmwv)
driver.SetInputMode(veh.SimpleDriver.InputMode.SPEED)
driver.SetTargetSpeed(10)  
driver.SetThrottleGrad(0.1)
driver.SetBrakeGrad(0.2)
driver.SetSteeringGrad(0.5)


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

    driver.Apply(timestep)
    system.DoStepDynamics(timestep)
    time += timestep

    if time > 10:
        driver.SetTargetSpeed(0) 
    
    
    camera_pos = hmmwv.GetPos() + chrono.ChVectorD(0, 5, -10)
    vis.GetCamera(0).SetPos(camera_pos)
    vis.GetCamera(0).SetLookAt(hmmwv.GetPos())