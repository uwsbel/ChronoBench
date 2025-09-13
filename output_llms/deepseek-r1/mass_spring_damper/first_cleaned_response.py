import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.1, 0.5)
    ground.AddVisualShape(ground_shape)
    
    
    mass = chrono.ChBody()
    mass.SetMass(1)
    mass.SetPos(chrono.ChVectorD(0, -1, 0))  
    mass.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    system.Add(mass)
    
    
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
    mass.AddVisualShape(box_shape)
    
    
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, True, 
                      chrono.ChVectorD(0, 0, 0),  
                      chrono.ChVectorD(0, 0, 0))   
    spring.SetSpringCoefficient(100)  
    spring.SetDampingCoefficient(1)   
    spring.SetRestLength(1.0)         
    system.Add(spring)
    
    
    spring_vis = chrono.ChSpringShape()
    spring_vis.SetNumRenderPoints(20)
    spring_vis.SetSpringRadius(0.05)
    spring.AddVisualShape(spring_vis)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Mass-Spring-Damper System')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1, -1, 1), chrono.ChVectorD(0, -1, 0))
    vis.AddTypicalLights()
    
    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()