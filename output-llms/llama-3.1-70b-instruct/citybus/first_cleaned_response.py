import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()


citybus = veh.CityBus()
citybus.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
citybus.SetChassisFixed(False)
citybus.SetInitPosition(chrono.ChVectorD(0, 0, 0.5))
citybus.SetInitOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
citybus.SetTireModel(veh.TireModelType.TM_RIGID_RING)


system.Add(citybus.GetChassisBody())
system.Add(citybus.GetChassisBodyAuxRef())


terrain = chrono.ChBodyEasyBox(system, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, 0, -0.5))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetTexture(chrono.ChTexture('./data/textures/terrain.jpg'))
system.Add(terrain)


visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(800, 600)
visualizer.SetWindowTitle('CityBus Simulation')
visualizer.SetCamera(chrono.ChVectorD(0, 0, 2), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
visualizer.SetFollowCamera(True, citybus.GetChassisBody(), chrono.ChVectorD(0, 0, 2))


driver = veh.ChIrrlichtDriver(citybus.GetChassisBody())
driver.SetSteeringMode(veh.SteeringModeType.SM_ARC)
driver.SetThrottleMode(veh.ThrottleModeType.TM_CONSTANT)
driver.SetBrakingMode(veh.BrakingModeType.BM_CONSTANT)
driver.SetSteeringGain(0.1)
driver.SetThrottleGain(0.1)
driver.SetBrakingGain(0.1)


while visualizer.Run():
    
    citybus.Synchronize(1 / 50.0)
    system.DoStepDynamics(1 / 50.0)

    
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.EndScene()

    
    driver.Synchronize(1 / 50.0)