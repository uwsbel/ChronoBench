import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import numpy as np




sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


chrono.SetChronoDataPath("./chrono_data/")  




terrain = veh.RigidTerrain(sys)
terrain.SetRoughness(0.1)
terrain.SetResolution(100, 100)
terrain.SetSize(20, 20)
terrain.CreatePatch(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix3d.Identity(), True)


rover = veh.Curiosity(sys)
rover.SetContactMethod(veh.ChContactMethod.SMC)
rover.SetChassisCollisionType(veh.ChassisCollisionType.BOX)
rover.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngAxis(0, chrono.ChVector3d(0, 1, 0))))
rover.Initialize()




sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))


time_step = 0.01




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(5, 5, -5))
vis.AddTypicalLights()



while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    driver = rover.GetDriver()
    if driver:
        driver.SetThrottle(0.5)  
        driver.SetSteering(0.0)  

    
    sys.DoStepDynamics(time_step)