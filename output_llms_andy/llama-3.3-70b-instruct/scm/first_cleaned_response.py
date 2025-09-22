import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


hmmwv = veh.HMMWV()
hmmwv.SetChassisFixed(False)
hmmwv.SetChassisPosition(chrono.ChVectorD(0, 1, 0))
hmmwv.SetChassisOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))
hmmwv.AddRigidTireWheels()
hmmwv.Visualize(chrono.ChVisualMode.MESH)


sys.Add(hmmwv)


terrain = veh.SCMDeformableTerrain()
terrain.SetSoilParameters(0.1, 0.5, 0.3, 0.2, 0.1)
terrain.SetMovingPatch(True, hmmwv.GetChassisBody(), 5, 5, 10)
terrain.Visualize(chrono.ChVisualMode.SINKAGE_FALSE_COLOR)


sys.Add(terrain)


driver = veh.InteractiveDriver()
driver.SetVehicle(hmmwv)
driver.SetSteeringGain(1.0)
driver.SetThrottleGain(1.0)
driver.SetBrakingGain(1.0)


sys.Add(driver)


step_size = 0.001
vis.SetWindowSize(1024, 768)
vis.SetCamera(chrono.ChVectorD(0, 10, -20), chrono.ChVectorD(0, 0, 0), 35)


while vis.Run():
    sys.DoStepDynamics(step_size)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    chrono.ChUtilities().Sleep(1 / 50.0)