fromchrono import *
fromchrono.core import *
fromchrono.irrlicht import *
fromchrono.shapes import *


chrono = Ch纪元()
irr = IrrlichtApp(chrono, 1280, 720, "PyChrono Mass-Spring-Damper System")


sys = ChSystemParallelNSC()


ground = ChBodyEasyBox(100, 100, 0.1, 1000, true, false, false)
ground.SetPos(ChVector<>(0, -10, 0))
sys.AddBody(ground)


mass = 1.0
radius = 0.5
height = 0.5


mass_obj = ChBodyEasyCylinder(radius, height, mass, false, false, false)
mass_obj.SetPos(ChVector<>(0, 0, 0))
mass_obj.SetBodyFixed(false)
sys.AddBody(mass_obj)


k = 1000.0  
c = 100.0   


spring = ChLinkTSDA()
spring.Initialize(mass_obj, ground, ChVector<>(0, 0, -height / 2), ChVector<>(0, 0, height / 2))
spring.SetK(k)
spring.SetC(c)
spring.SetDampingType(ChLinkTSDA.DAMPING_TYPE_LINEAR)
sys.AddLink(spring)


irr.SetBackgroundColor(ChColor(0.5, 0.5, 0.5))
irr.AddTypicalLights()
irr.SetCameraPosition(ChVector<>(0, 20, 20))
irr.SetCameraTarget(ChVector<>(0, 0, 0))


irr.AddSystem(sys)
irr.StartSimulation(10000, 1e-3)