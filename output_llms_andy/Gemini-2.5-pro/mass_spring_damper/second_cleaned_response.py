import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


class MySpringForce(chrono.ForceFunctor):
    def __init__(self, k_val, r_val):
        super().__init__()
        self.k = k_val  
        self.r = r_val  

    def evaluate(self, time, rest_length_val, length, vel, link):
        
        force = self.k * (length - rest_length_val) + self.r * vel
        return force


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)



sph_1_vis = chrono.ChSphereShape(0.1)
sph_1_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.5)) 
ground.AddVisualShape(sph_1_vis, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))


sph_2_vis = chrono.ChSphereShape(0.1)
sph_2_vis.SetColor(chrono.ChColor(0.2, 0.2, 0.5)) 
ground.AddVisualShape(sph_2_vis, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))





body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1)) 



box_1_vis = chrono.ChBoxShape(0.5, 0.5, 0.5) 
box_1_vis.SetColor(chrono.ChColor(0.6, 0, 0)) 
body_1.AddVisualShape(box_1_vis)


spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)



spring_1_vis_asset = chrono.ChSpringShape(0.05, 80, 15)
spring_1.AddVisualShape(spring_1_vis_asset)





body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0)) 
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1) 
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1)) 


box_2_vis = chrono.ChBoxShape(0.5, 0.5, 0.5) 
box_2_vis.SetColor(chrono.ChColor(0, 0.6, 0)) 
body_2.AddVisualShape(box_2_vis)


spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0))
spring_2.SetRestLength(rest_length)


custom_spring_force = MySpringForce(spring_coef, damping_coef)
spring_2.RegisterForceFunctor(custom_spring_force)
sys.AddLink(spring_2)


spring_2_vis_asset = chrono.ChSpringShape(0.05, 80, 15)
spring_2.AddVisualShape(spring_2_vis_asset)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA with Custom Force Functor Demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -1, 6)) 
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)