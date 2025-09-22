import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())


system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)


ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


rover = veh.Curiosity(system)
rover.SetDriver(veh.CuriosityDriver())
rover.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, -0.5, 0), chrono.ChQuaternionD(1, 0, 0, 0)))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()


vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 3), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights(chrono.ChVectorD(30, 30, 30), chrono.ChVectorD(-30, -30, 30), 200, 200)
vis.AddLightWithShadow(chrono.ChVectorD(10, 20, 10), chrono.ChVectorD(0, 0, 0), 50, 5, 40, 80, 512)
vis.EnableShadows()


driver = veh.ChIrrGuiDriver(vis)
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.02)


while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = driver.GetSteering()
    driver_inputs.m_throttle = driver.GetThrottle()
    driver_inputs.m_braking = driver.GetBraking()
    
    rover.DriverInputs = driver_inputs
    rover.Update(time)
    
    
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis, 1, 1, 12, 12, 
                      chrono.ChCoordsysD(chrono.ChVectorD(0, -1, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
                      chrono.ChColor(0.5, 0.5, 0.5), True)
    chronoirr.drawAllCOGs(vis, 1.0)
    vis.EndScene()
    
    
    system.DoStepDynamics(1e-3)