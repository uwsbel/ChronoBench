import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.terrain as terr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


terrain = terr.ChTerrainSCM()
terrain.Create(system, 100, 100, 1, 1)  
terrain.SetLiquidDepth(0.1)
terrain.SetMu(0.5)
terrain.SetCohesion(1000)
terrain.SetFriction(0.8)


terrain.Set_K_arr(1000000)
terrain.Set_K_dem(1000000)
terrain.Set_K_tan(100000)
terrain.Set_K_visc(1000)
terrain.Set_damping(0.1)


terrain.SetMovingPatch(True)
terrain.SetPatchRadius(1.5)


terrain.SetVisualizeSinkage(True)

system.Add(terrain)


hmmwv = veh.ChHMMWV()
hmmwv.SetVehicleModel(veh.VehicleModel.HMMWV)
hmmwv.SetTireType(veh.TireModelType.RIGID)
hmmwv.SetTireRadius(0.4)
hmmwv.SetTireWidth(0.3)


hmmwv.SetPos(chrono.ChVectorD(0, 1, 0))
hmmwv.SetTransform(chrono.ChQuaternionD(chrono.ChVectorD(0, 1, 0), chrono.CH_C_DEG * 0))


hmmwv.SetBodyVisualization(True)
hmmwv.SetWheelVisualization(True)

system.Add(hmmwv)


driver = veh.ChIrrVehicleDriver(hmmwv)
driver.SetSteeringDelta(10)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV on SCM Terrain')
vis.Initialize()
vis.AddTypicalCamera(chrono.ChVectorD(0, 10, -10), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


time_step = 0.005
vis.SetUpdateInterval(time_step)
vis.SetRenderMode(chronoirr.ChVisualSystemIrrlicht.RenderMode.RENDER_MODE_SOLID)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    system.DoStepDynamics(time_step)

    
    driver.Synchronize(time_step)