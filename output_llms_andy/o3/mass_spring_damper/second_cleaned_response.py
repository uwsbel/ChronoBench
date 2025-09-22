import pychrono as chrono
import pychrono.irrlicht as chronoirr




rest_length   = 1.5          
spring_coef   = 50.0         
damping_coef  = 1.0          
time_step     = 1.0e-3       




sys = chrono.ChSystemNSC()


try:                                     
    sys.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))
except AttributeError:
    sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))




ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetCollide(False)
sys.AddBody(ground)


marker_rad = 0.10

sph_1 = chrono.ChVisualShapeSphere(marker_rad)
ground.AddVisualShape(sph_1,
                      chrono.ChFrameD(chrono.ChVectorD(-1, 0, 0)))   

sph_2 = chrono.ChVisualShapeSphere(marker_rad)
ground.AddVisualShape(sph_2,
                      chrono.ChFrameD(chrono.ChVectorD( 1, 0, 0)))   




def make_body(pos, color):
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(*pos))
    body.SetMass(1.0)
    body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    body.SetCollide(False)

    box = chrono.ChVisualShapeBox(1, 1, 1)        
    box.SetColor(color)
    body.AddVisualShape(box)

    sys.AddBody(body)
    return body


body_1 = make_body(pos = (-1, -3, 0), color = chrono.ChColor(0.6, 0.0, 0.0))
body_2 = make_body(pos = ( 1, -3, 0), color = chrono.ChColor(0.0, 0.0, 0.6))




spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True,
                    chrono.ChVectorD(0, 0, 0),     
                    chrono.ChVectorD(-1, 0, 0))    
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, rest_length))
sys.AddLink(spring_1)




class MySpringForce(chrono.ForceFunctor):
    
    def __init__(self, k, c):
        chrono.ForceFunctor.__init__(self)
        self.k = k
        self.c = c

    def __call__(self, time, rest_length, length, vel, link):
        return -self.k * (length - rest_length) - self.c * vel

user_force = MySpringForce(spring_coef, damping_coef)




spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True,
                    chrono.ChVectorD(0, 0, 0),     
                    chrono.ChVectorD( 1, 0, 0))    
spring_2.SetRestLength(rest_length)
spring_2.RegisterForceFunctor(user_force)
spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, rest_length))
sys.AddLink(spring_2)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("ChLinkTSDA demo with custom force functor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()




while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)