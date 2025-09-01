import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Custom force functor class
class MySpringForce(chrono.ChFunction):
    def __init__(self, rest_length, spring_coef, damping_coef):
        chrono.ChFunction.__init__(self)
        self.rest_length = rest_length
        self.spring_coef = spring_coef
        self.damping_coef = damping_coef

    def GetForce(self, x1, x2):
        r = x2 - x1
        if r.GetMagnitude() < self.rest_length:
            return chrono.ChVector3d(0, 0, 0)
        else:
            return (self.spring_coef * (r - self.rest_length).normalized() + self.damping_coef * r.normalized()) * r


# Create the spring using the custom force functor
def create_spring_2(body_2, ground, rest_length, spring_coef, damping_coef):
    spring_2 = chrono.ChLinkLockable()
    spring_2.Initialize(body_2, ground, True, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)), chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))
    my_force = MySpringForce(rest_length, spring_coef, damping_coef)
    spring_2.GetSpring().SetFunction(my_force)
    return spring_2


# Create the spring using direct spring and damping coefficients
def create_spring_1(body_1, ground, rest_length, spring_coef, damping_coef):
    spring_1 = chrono.ChLinkTSDA()
    spring_1.Initialize(body_1, ground, True, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(-1, 0, 0))
    spring_1.SetRestLength(rest_length)
    spring_1.SetSpringCoefficient(spring_coef)
    spring_1.SetDampingCoefficient(damping_coef)
    return spring_1


# Main function
def main():
    #  Create the system
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))

    # Create the ground body
    ground = chrono.ChBody()
    sys.AddBody(ground)
    ground.SetFixed(True)
    ground.EnableCollision(False)

    # Create a visual shape for the ground
    sph_1 = chrono.ChVisualShapeSphere(0.1)
    ground.AddVisualShape(sph_1, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

    # Create the first body
    body_1 = chrono.ChBody()
    sys.AddBody(body_1)
    body_1.SetPos(chrono.ChVector3d(-1, -3, 0))
    body_1.SetFixed(False)
    body_1.EnableCollision(False)
    body_1.SetMass(1)
    body_1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

    # Attach a visualization asset
    box_1 = chrono.ChVisualShapeBox(1, 1, 1)
    box_1.SetColor(chrono.ChColor(0.6, 0, 0))
    body_1.AddVisualShape(box_1)

    # Create the spring between body_1 and ground
    spring_1 = create_spring_1(body_1, ground, 1.5, 50, 1)
    sys.AddLink(spring_1)

    # Attach a visualization asset
    spring_1.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

    # Create the second body
    body_2 = chrono.ChBody()
    sys.AddBody(body_2)
    body_2.SetPos(chrono.ChVector3d(1, -3, 0))
    body_2.SetFixed(False)
    body_2.EnableCollision(False)
    body_2.SetMass(1)
    body_2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))

    # Attach a visualization asset
    box_2 = chrono.ChVisualShapeBox(1, 1, 1)
    box_2.SetColor(chrono.ChColor(0.6, 0, 0))
    body_2.AddVisualShape(box_2)

    # Create the spring between body_2 and ground using the custom force functor
    spring_2 = create_spring_2(body_2, ground, 1.5, 50, 1)
    sys.AddLink(spring_2)

    # Attach a visualization asset
    spring_2.AddVisualShape(chrono.ChVisualShapeSpring(0.05, 80, 15))

    # Create the Irrlicht interface
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('ChLinkTSDA demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 0, 6))
    vis.AddTypicalLights()

    # Run the simulation
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)


if __name__ == "__main__":
    main()