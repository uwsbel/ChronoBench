import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


my_application = chronoirr.ChIrrApp(chronoirr.NullMaterial(), chronoirr.COREDLL_AUTODETECT, "CityBus Simulation")


my_application.SetWindowSize(800, 600)
my_application.SetWindowPosition(50, 50)


my_system = chrono.ChSystemNSC()


bus = chrono.ChBodyEasyCylinder(1.5, 3, 1000, True, True)
bus.SetPos(chrono.ChVectorD(0, 0, 1))
bus.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
my_system.Add(bus)


tire = chrono.ChTire()
tire.SetTireType(chrono.ChTire::Type::CYLINDER)
tire.SetTireRadius(0.5)
tire.SetTireWidth(0.2)
tire.SetTireSteering(0)
tire.SetTireFriction(0.9)
tire.SetTireGrip(0.9)
tire.SetTireDamping(0.1)
tire.SetTireStiffness(10000)


terrain = chrono.ChRigidTerrain(my_system.GetGroundBody())
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/grass.png"))
terrain.Add(chrono.ChBoxShape(chrono.ChVectorD(10, 10, 1), chrono.ChVectorD(0, 0, 0)))
terrain.Initialize()


bus.AddAsset(tire)


my_application.AddTypicalCamera(chronoirr.Vec(0, 0, 5), chronoirr.Vec(0, 0, 0))


driver = chrono.ChDriver::Inputs()
driver.SetSteeringDelay(0.1)
driver.SetThrottle(0.5)
driver.SetBraking(0)


my_system.SetTimestep(0.01)


while my_application.GetDevice().run():
    
    my_system.DoStepDynamics(0.02)

    
    driver.SetSteering(my_application.GetKeyboard().GetKeyState(chronoirr.KEY_KEY_A) - my_application.GetKeyboard().GetKeyState(chronoirr.KEY_KEY_D))
    driver.SetThrottle(my_application.GetKeyboard().GetKeyState(chronoirr.KEY_KEY_W))
    driver.SetBraking(my_application.GetKeyboard().GetKeyState(chronoirr.KEY_KEY_S))

    
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()