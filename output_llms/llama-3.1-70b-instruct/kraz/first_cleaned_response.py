import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()
sys.SetSolverType(chrono.ChSolver.Type_Acceleration)


kraz = veh.Kraz()
kraz.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
kraz.SetChassisFixed(False)
sys.Add(kraz.GetPart(0))


kraz.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
kraz.SetChassisOrientation(chrono.Q_from_AngX(chrono.CH_C_PI_2))
kraz.SetChassisVelocity(chrono.ChVectorD(0, 0, 0))
kraz.SetChassisAcceleration(chrono.ChVectorD(0, 0, 0))


terrain = chrono.ChBodyEasyBox(sys, 100, 10, 100, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -10, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetFriction(0.9)
terrain.SetRestitution(0.1)
sys.Add(terrain)


driver = veh.ChIrrlichtDriver()
driver.SetVehicle(kraz)
driver.SetSteeringDelta(chrono.CH_C_PI / 16)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.1)
sys.Add(driver.GetPart(0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Kraz Simulation")
vis.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0))
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddLogo()
vis.AddCameraVerticalAll()
vis.SetShowFPS(True)
vis.SetShowInfo(True)


tstep = 0.01
tend = 10
while sys.GetChTime() < tend:
    sys.DoStepDynamics(tstep)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    chrono.ChUtils::Sleep(tstep)


vis.Finalize()