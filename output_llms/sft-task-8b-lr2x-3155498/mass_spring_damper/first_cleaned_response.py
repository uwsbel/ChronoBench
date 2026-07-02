import pychrono as chrono
import math


def create_mass_spring_damper_system():
    
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    ground = chrono.ChBodyEasyBox(5, 1, 0.1, 1000, True, True)
    ground.SetName("Ground")
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, -0.5, 0))
    system.Add(ground)

    
    mass = chrono.ChBodyEasySphere(0.15, 1000, True, True)
    mass.SetName("Mass")
    mass.SetMass(1.0)
    mass.SetInertiaXX(chrono.ChVector3d(0.01, 0.01, 0.01))
    mass.SetPos(chrono.ChVector3d(0, 1.0, 0))
    mass.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.1, 0.1))
    system.Add(mass)

    
    spring = chrono.ChLinkTSDA()
    spring.SetName("Spring-Damper")
    spring.Initialize(mass, ground, True, chrono.ChVector3d(0, 1.0, 0), chrono.ChVector3d(0, -0.5, 0))
    spring.SetRestLength(1.5)
    spring.SetSpringCoefficient(100.0)
    spring.SetDampingCoefficient(10.0)
    system.AddLink(spring)

    
    spring_shape = chrono.ChVisualShapeSpring(1.5, 0.03, 60)
    spring_shape.SetColor(chrono.ChColor(0.0, 0.4, 0.8))
    spring.AddVisualShape(spring_shape)

    
    mass_shape = chrono.ChVisualShapeSphere(0.15)
    mass_shape.SetColor(chrono.ChColor(0.8, 0.1, 0.1))
    mass.AddVisualShape(mass_shape)

    return system, ground, mass, spring


def create_irrlicht_visualization(system):
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 720)
    vis.SetWindowTitle("PyChrono Mass-Spring-Damper System")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0, 2.5, 3), chrono.ChVector3d(0, 0.5, 0))
    vis.AddTypicalLights()
    return vis


def run_simulation(duration, step_size):
    system, ground, mass, spring = create_mass_spring_damper_system()
    vis = create_irrlicht_visualization(system)

    while vis.Run() and system.GetChTime() < duration:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(step_size)


if __name__ == "__main__":
    run_simulation(2.0, 0.001)