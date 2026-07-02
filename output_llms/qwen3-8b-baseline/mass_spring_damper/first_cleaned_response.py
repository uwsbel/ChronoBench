import pychrono as chrono
from pychrono import irrlicht
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, -9.81))


ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(10, 10, 0.1)
ground.GetCollisionModel().Build()
system.Add(ground)


mass = chrono.ChBody()
mass.SetMass(1)
mass.SetPos(chrono.ChVectorD(0, 0, 0.5))
mass.SetBodyFixed(False)
mass.GetCollisionModel().ClearModel()
mass.GetCollisionModel().AddSphere(0.5)
mass.GetCollisionModel().Build()
system.Add(mass)


mass.AddVisualShape(chrono.ChVisualShape(chrono.ChVisualShape.SHAPE_SPHERE, 0.5))


link = chrono.ChLinkTSDA()
link.SetBodyA(ground)
link.SetBodyB(mass)
link.SetPointA(chrono.ChVectorD(0, 0, 0))
link.SetPointB(chrono.ChVectorD(0, 0, 0.5))
link.SetSpringStiffness(1000)
link.SetDampingCoefficient(10)
system.Add(link)


spring_line = chrono.ChLineShape()
spring_line.SetLine(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 0.5))
system.Add(spring_line)


application = irrlicht.ChIrrApp(system, "Mass-Spring-Damper", irrlicht.ChVectorD(0, 0, 0), irrlicht.ChVectorD(10, 10, 10))
application.AssetBindAll()
application.AssetUpdateAll()


application.SetCameraPosition(chrono.ChVectorD(0, 2, 2))
application.SetCameraLookAtPoint(chrono.ChVectorD(0, 0, 0.5))


application.AddLightPoint(chrono.ChVectorD(0, 0, 0), 10, 1, 1, 1)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()