import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


class MySpringForce(chrono.ChLinkTSDA.ForceFunctor):
    def __init__(self, k, c):
        super().__init__()
        self.k = k
        self.c = c
        
    def Evaluate(self, time, rest_length, length, vel, link):
        force = -self.k * (length - rest_length) - self.c * vel
        return force


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


ground = chrono.ChBodyEasyBox(1, 1, 1, 1000)  
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


sph_1 = chrono.ChVisualShapeSphere(0.1)
sph_1.SetColor(chrono.ChColor(0, 1, 0))  
ground.AddVisualShape(sph_1, chrono.ChVector3d(-1, 0, 0))

sph_2 = chrono.ChVisualShapeSphere(0.1)
sph_2.SetColor(chrono.ChColor(0, 1, 0))  
ground.AddVisualShape(sph_2, chrono.ChVector3d(1, 0, 0))


body_1 = chrono.ChBodyEasyBox(1, 1, 1, 1000)  
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)


box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))  
body_1.AddVisualShape(box_1)


spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, 
                    chrono.ChVector3d(0, 0, 0), 
                    chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 80))  


body_2 = chrono.ChBodyEasyBox(1, 1, 1, 1000)  
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)


box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))  
body_2.AddVisualShape(box_2)


spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, 
                    chrono.ChVector3d(0, 0, 0), 
                    chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)
spring_2.SetForceFunctor(MySpringForce(spring_coef, damping_coef))
sys.AddLink(spring_2)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 15, 80))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Custom Spring Functor Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -6, 9))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)