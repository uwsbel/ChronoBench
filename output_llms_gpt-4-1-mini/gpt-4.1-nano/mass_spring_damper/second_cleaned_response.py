import pychrono as chrono
import pychrono.irrlicht as chronoirr


rest_length = 1.5
spring_coef = 50
damping_coef = 1


class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coefficient, damping_coefficient):
        super().__init__()
        self.spring_coefficient = spring_coefficient
        self.damping_coefficient = damping_coefficient
        self.spring_force = 0
        self.damping_force = 0

    def evaluate(self, context):
        
        delta_length = context.GetDisplacement()  
        relative_velocity = context.GetVelocity()  

        self.spring_force = -self.spring_coefficient * delta_length
        self.damping_force = -self.damping_coefficient * relative_velocity
        total_force = self.spring_force + self.damping_force
        return total_force


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))


ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)


sph_1 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_1, chrono.ChFrameD(chrono.ChVector3d(-1, 0, 0)))


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


spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)


spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))



body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))


box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0, 0, 0.6))
body_2.AddVisualShape(box_2)



sph_2 = chrono.ChVisualShapeSphere(0.1)
ground.AddVisualShape(sph_2, chrono.ChFrameD(chrono.ChVector3d(1, 0, 0)))





class SpringForceContext:
    def __init__(self, body, ground, attachment_point_body, attachment_point_ground):
        self.body = body
        self.ground = ground
        self.attachment_point_body = attachment_point_body
        self.attachment_point_ground = attachment_point_ground

    def GetDisplacement(self):
        
        p1 = self.body.GetFrameAbs().TransformPoint(self.attachment_point_body)
        p2 = self.ground.GetFrameAbs().TransformPoint(self.attachment_point_ground)
        return (p1 - p2).Length()

    def GetVelocity(self):
        
        v1 = self.body.GetFrameVel()
        v2 = self.ground.GetFrameVel()
        p1 = self.body.GetFrameAbs().TransformPoint(self.attachment_point_body)
        p2 = self.ground.GetFrameAbs().TransformPoint(self.attachment_point_ground)
        v1_world = v1 + self.body.GetFrameRef().GetVelocityP(p1 - self.body.GetFrameAbs().GetPos())
        v2_world = v2 + self.ground.GetFrameRef().GetVelocityP(p2 - self.ground.GetFrameAbs().GetPos())
        relative_v = v1_world - v2_world
        
        delta = p1 - p2
        delta_norm = delta.Length()
        if delta_norm > 1e-6:
            dir = delta / delta_norm
            return relative_v.Dot(dir)
        else:
            return 0.0


my_spring_force_functor = MySpringForce(spring_coefficient=spring_coef,
                                         damping_coefficient=damping_coef)


spring_2 = chrono.ChLinkSpring()
spring_2.Initialize(body_2, ground, False, 
                    chrono.ChVectorD(0,0,0),  
                    chrono.ChVectorD(0,0,0)   
                   )
spring_2.SetSpringRestLength(rest_length)
spring_2.SetSpringCoefficient(spring_coef)
spring_2.SetDampingCoefficient(damping_coef)


spring_2.RegisterForceFunctor(my_spring_force_functor)

sys.AddLink(spring_2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA and Custom Spring demo')
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