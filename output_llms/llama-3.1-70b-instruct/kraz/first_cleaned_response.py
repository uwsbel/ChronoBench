import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
solver = chrono.ChSolverBB()
mysystem.SetSolver(solver)


kraz = chrono_vehicle.ChKraz()
kraz.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
kraz.SetChassisCollisionShape(chrono.ChBox(1, 1, 1))
kraz.SetChassisFixed(False)
mysystem.Add(kraz)


kraz.SetChassisPosition(chrono.ChVectorD(0, 0.5, 0))
kraz.SetChassisOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
kraz.SetChassisVelocity(chrono.ChVectorD(10, 0, 0))
kraz.SetChassisAcceleration(chrono.ChVectorD(0, 0, 0))


terrain = chrono.ChBodyEasyBox(mysystem, 10, 1, 10, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetFriction(0.7)
terrain.SetRestitution(0.7)
mysystem.Add(terrain)


driver = chrono_vehicle.ChIrrlichtDriver()
driver.SetVehicle(kraz)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)
mysystem.Add(driver)


visual_system = chronoirr.ChVisualSystemIrrlicht()
visual_system.SetWindowSize(1024, 768)
visual_system.SetWindowTitle('Kraz Simulation')
visual_system.SetCamera(chrono.ChVectorD(0, 1, -3), chrono.ChVectorD(0, 1, 0))
visual_system.SetCameraUp(chrono.ChVectorD(0, 1, 0))
visual_system.AddTypicalLights()
visual_system.AddSkyBox()
visual_system.AddLogo()
mysystem.Add(visual_system)


tstep = 0.01
tend = 10
while mysystem.GetChTime() < tend:
    mysystem.DoStepDynamics(tstep)
    driver.Synchronize(tstep)
    visual_system.BeginScene()
    visual_system.Render()
    visual_system.EndScene()
    chrono.ChUtils::Sleep(tstep)