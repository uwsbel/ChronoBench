import chrono as ch
import chrono.irrlicht as irr


ch.CH_INIT()


sys = ch.ChSystemNSC()


ground = ch.ChBody()
ground.SetPos(ch.ChVector3d(0, 0, 0))
sys.Add(ground)


mass = ch.ChBody()
mass.SetMass(1.0)
mass.SetInertiaXX(ch.ChVector3d(1, 1, 1))
mass.SetPos(ch.ChVector3d(1, 1, 0))
sys.Add(mass)


link = ch.ChLinkTSDA()
link.SetName('damper')
link.Initialize(ground, mass, 
                ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)), 
                ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(1, 0, 0, 0)))
link.RegisterVisualization(True)
link.SetSpringConstant(1000.0)  
link.SetDampingCoefficient(10.0)  
sys.Add(link)


mass_shape = ch.ChSphereShape(0.1)
mass.AddVisualShape(mass_shape)

spring_shape = ch.ChLineShape()
spring_shape.SetNumSegments(10)
spring_shape.SetSegmentPositions(link.GetPointsInBodyA(), link.GetPointsInBodyB())
link.AddVisualShape(spring_shape)


app = irr.ChIrrApp(sys, 'Mass-Spring-Damper System', 800, 600)


app.SetCameraPos(ch.ChVector3d(0, -5, 0))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))


app.EnableDefaultLights()


while app.Run():
    sys.Update()