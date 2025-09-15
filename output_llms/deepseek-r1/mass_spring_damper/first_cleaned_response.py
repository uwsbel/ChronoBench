import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground_pos = chrono.ChVectorD(0, 0, 0)
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
    ground_shape.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    ground.AddVisualShape(ground_shape)
    system.Add(ground)
    
    
    mass = chrono.ChBody()
    mass.SetMass(1.0)
    initial_pos = chrono.ChVectorD(0, 0, -1)  
    mass.SetPos(initial_pos)
    
    
    mass_shape = chrono.ChSphereShape()
    mass_shape.GetSphereGeometry().rad = 0.2
    mass_shape.SetColor(chrono.ChColor(0.8, 0.3, 0.2))
    mass.AddVisualShape(mass_shape)
    system.Add(mass)
    
    
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, True, ground_pos, initial_pos)
    
    
    spring.SetSpringCoefficient(50)   
    spring.SetDampingCoefficient(1)   
    spring.SetRestLength(0.5)         
    
    
    spring_shape = chrono.ChSpringShape()
    spring_shape.SetNumRenderPoints(50)
    spring_shape.SetColor(chrono.ChColor(0.1, 0.9, 0.2))
    spring.AddVisualShape(spring_shape)
    system.Add(spring)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Mass-Spring-Damper System')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1, 0.5, 0), chrono.ChVectorD(0, 0, -1))
    vis.AddTypicalLights()
    
    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()