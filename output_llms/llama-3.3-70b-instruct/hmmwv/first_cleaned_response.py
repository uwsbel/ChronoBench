import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 100))
vis.SetCameraAZEL(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0))
vis.SetLogo("")


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetTireModel(veh.TireModel_TMEASY)
hmmwv.SetLocation(chrono.ChVectorD(0, 0, 1.5))
hmmwv.SetRotation(chrono.Q_from_AngX(chrono.CH_C_PI_2))
sys.Add(hmmwv)


hmmwv.AddVisualization(veh.Visualization_PRIMITIVES)


terrain = chrono.ChBodyEasyBox(sys, 100, 100, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1.5, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
sys.Add(terrain)


texture = chronoirr.ChTexture()
texture.SetTexture(chronoirr.GetTexture("terrain.png"))
vis.AddTexture(terrain, texture)


driver = veh.DriverInteractive(hmmwv)
driver.SetSteeringGain(0.5)
driver.SetThrottleGain(0.5)
driver.SetBrakingGain(0.5)


while vis.Run():
    
    sys.DoStepDynamics(0.02)
    
    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    
    
    driver.Synchronize(0.02)
    
    
    chrono.ChSleep(0.02)