import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
my_system.Add(ground)


pendulum = chrono.ChBodyEasySphere(0.5, 1000, True, True)
pendulum.SetPos(chrono.ChVectorD(0, 5, 0))
my_system.Add(pendulum)


joint = chrono.ChLinkLockRevolute()
joint.Initialize(ground, pendulum, chrono.ChCoordsysD(chrono.ChVectorD(0, 5, 0)))
my_system.Add(joint)


pendulum.SetWvel_par(chrono.ChVectorD(0, 0, 3))


my_application = irr.ChIrrApp(my_system, 'Pendulum Simulation', irr.dimension2du(1024, 768))


my_application.AddTypicalLights()
my_application.AddCamera(irr.vector3df(0, 10, -20))
my_application.AssetBindAll()
my_application.AssetUpdateAll()


while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()

    
    print("Position: ", pendulum.GetPos())
    print("Velocity: ", pendulum.GetWvel_par())

    
    chrono.ChTime.Sleep(0.01)