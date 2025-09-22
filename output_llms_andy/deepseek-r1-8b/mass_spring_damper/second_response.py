import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Custom Force Functor
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def Evaluate(self, body1, body2, point_on1, point_on2):
        force = chrono.ChVector3d(0, 0, 0)
        # Calculate spring force
        spring_vector = body1.GetPos() - body2.GetPos()
        spring_length = spring_vector.Length()
        if spring_length > 1e-10:  # Avoid division by zero
            spring_force = self.spring_coef * (spring_length - self.rest_length) * unit_vector
            force = spring_force
        # Calculate damping force
        velocity1 = body1.GetVelocity()
        velocity2 = body2.GetVelocity()
        relative_velocity = velocity1 - velocity2
        damping_force = -self.damping_coef * spring_vector.Length() * relative_velocity
        force += damping_force
        return force

rest_length = 1.5
spring_coef = 50
damping_coef = 1

sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetFixed(True)
ground.EnableCollision(False)

# Visual elements for ground
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

# Visual elements for body_1
box_1 = chrono.ChVisualShapeBox(1, 1, 1)
box_1.SetColor(chrono.ChColor(0.6, 0, 0))
body_1.AddVisualShape(box_1)

# Create the first spring
spring_1 = chrono.ChLinkTSDA()
spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_1.SetRestLength(rest_length)
spring_1.SetSpringCoefficient(spring_coef)
spring_1.SetDampingCoefficient(damping_coef)
sys.AddLink(spring_1)

# Visual elements for spring_1
spring_visual_1 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_1.AddVisualShape(spring_visual_1)

# Create body_2
body_2 = chrono.ChBody()
sys.AddBody(body_2)
body_2.SetPos(chrono.ChVector3d(1, -3, 0))
body_2.SetFixed(False)
body_2.EnableCollision(False)
body_2.SetMass(1)
body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

# Visual elements for body_2
box_2 = chrono.ChVisualShapeBox(1, 1, 1)
box_2.SetColor(chrono.ChColor(0.6, 0, 0))
body_2.AddVisualShape(box_2)

# Create the second spring using the custom force functor
class MySpringForce(chrono.ForceFunctor):
    def __init__(self, spring_coef, damping_coef):
        super().__init__()
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def Evaluate(self, body1, body2, point_on1, point_on2):
        force = chrono.ChVector3d(0, 0, 0)
        # Calculate spring force
        spring_vector = body1.GetPos() - body2.GetPos()
        spring_length = spring_vector.Length()
        if spring_length > 1e-10:  # Avoid division by zero
            spring_force = self.spring_coef * (spring_length - self.rest_length) * unit_vector
            force = spring_force
        # Calculate damping force
        velocity1 = body1.GetVelocity()
        velocity2 = body2.GetVelocity()
        relative_velocity = velocity1 - velocity2
        damping_force = -self.damping_coef * spring_vector.Length() * relative_velocity
        force += damping_force
        return force

spring_2 = chrono.ChLinkTSDA()
spring_2.Initialize(body_2, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
spring_force = MySpringForce(spring_coef, damping_coef)
spring_2.SetForceFunctor(spring_force)
spring_2.SetRestLength(rest_length)
sys.AddLink(spring_2)

# Visual elements for spring_2
spring_visual_2 = chrono.ChVisualShapeSpring(0.05, 80, 15)
spring_2.AddVisualShape(spring_visual_2)

# Visual system setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ChLinkTSDA demo with Custom Force Functor')
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