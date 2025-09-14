import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

    
    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.ChVector3d(0, 0, 0))
    ground_box = chrono.ChBoxShape(chrono.ChVector3d(1, 0.5, 1))
    ground_box.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
    ground.AddVisualShape(ground_box)
    system.Add(ground)

    
    mass = chrono.ChBody()
    mass.SetPos(chrono.ChVector3d(0, 2, 0))
    mass.SetMass(1.0)
    mass_sphere = chrono.ChSphereShape(0.2)
    mass_sphere.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    mass.AddVisualShape(mass_sphere)
    system.Add(mass)

    
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, 
                     chrono.ChVector3d(0, 0, 0),  
                     chrono.ChVector3d(0, 0, 0))   
    spring.SetSpringCoefficient(100)  
    spring.SetDampingCoefficient(5)   
    system.Add(spring)

    
    spring_shape = chrono.ChSpringShape()
    spring_shape.SetSpringRadius(0.1)
    spring_shape.SetNumCoils(8)
    spring_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    spring.AddVisualShape(spring_shape)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Mass-Spring-Damper System')
    vis.Initialize()
    vis.AddCamera(chrono.ChVector3d(0, 3, 6), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()
    vis.AddSkyBox()

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()