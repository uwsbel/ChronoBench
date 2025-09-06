import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Mass-Spring-Damper Simulation')
    vis.Initialize()
    vis.AddCamera(chronoirr.ChVectorD(0, 2, 2))  
    vis.AddTypicalLights()

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    
    ground_shape = chrono.ChVisualShapeBox(0.1, 0.1, 0.1)
    ground.AddVisualShape(ground_shape)
    sys.Add(ground)

    
    mass = chrono.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, True, True)
    mass.SetPos(chrono.ChVectorD(0, 0, 0.5))
    sys.Add(mass)

    
    springdamper = chrono.ChLinkTSDA()
    springdamper.Initialize(ground, mass, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    springdamper.SetSpringK(100)      
    springdamper.SetDampingR(10)     
    springdamper.SetRestLength(0.5)  
    sys.AddLink(springdamper)

    
    spring_shape = chrono.ChVisualShapeCylinder(0.02, 0.5)  
    spring_shape.SetColor(chrono.ChColor(1, 0, 0))  
    sys.AddVisualShape(spring_shape)  

    
    time_step = 0.001

    
    while vis.Run():
        vis.BeginScene()
        
        
        ground_pos = ground.GetPos()
        mass_pos = mass.GetPos()
        delta = mass_pos - ground_pos
        length = delta.Length()
        axis = delta / length if length > 0 else chrono.ChVectorD(0,0,1)
        midpoint = (ground_pos + mass_pos) * 0.5
        
        
        rotation = chrono.ChQuaternionD()
        rotation.QlookAtDir(delta, chrono.ChVectorD(0, 1, 0))  
        
        
        spring_shape.SetPos(midpoint)
        spring_shape.SetRot(rotation)
        scale_factor = length / 0.5  
        spring_shape.SetScale(chrono.ChVectorD(1, 1, scale_factor))
        
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()