import pychrono as chrono
import pychrono.irrlicht as chronoirr

rest_length = 1.5
spring_coef = 50
damping_coef = 1

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFramed(chrono.ChVector3d(1, 0, 0)))


body_1 = chrono.ChBody()
sys.AddBody(body_1)
body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
body_1.SetFixed(False)
body_1.EnableCollision(False)
body_1.SetMass(1)
body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)


body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0.6, 0))
body_2.AddVisualShape(box_2)


class MySpringForce(chrono.ForceFunctor):
    def __init__(self, body, ground, body_point, ground_point, spring_coef, damping_coef):
        super().__init__()
        self.body = body
        self.ground = ground
        self.body_point = body_point
        self.ground_point = ground_point
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def Evaluate(self, time, body, pos, vel, accel):
        
        body_pos = body.GetPos()
        body_attachment = body_pos + self.body_point
        
        ground_pos = self.ground.GetPos()
        ground_attachment = ground_pos + self.ground_point
        
        displacement = body_attachment - ground_attachment
        distance = displacement.Length()
        
        spring_force = -self.spring_coef * (distance - rest_length) * displacement.Normalize()
        
        body_vel = body.GetPos_dt()
        damping_force = -self.damping_coef * body_vel
        
        total_force = spring_force + damping_force
        return total_force


spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)


my_spring_force = MySpringForce(body_2, ground, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(1, 0, 0), spring_coef, damping_coef)
body_2.AddForce(my_spring_force)


spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)