import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath("/path/to/your/chrono/data")  


my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(2, 2, 0.1, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, 0, -0.1))
my_system.Add(ground)


my_body = chrono.ChBodyEasySphere(0.1, 10, True, True)
my_body.SetPos(chrono.ChVectorD(0, 0, 1))
my_system.Add(my_body)


my_body.SetBodyFixed(False)


my_application = chronoirr.ChIrrApp(my_system, "PyChrono Simulation", chronoirr.dimension2du(800, 600))


my_application.Run()