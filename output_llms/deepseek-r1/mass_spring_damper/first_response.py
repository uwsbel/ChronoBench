import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system and set gravity
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # No gravity for pure vertical motion
    
    # 2. Create ground body
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)
    
    # Add visualization to ground
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.1, 0.5)
    ground.AddVisualShape(ground_shape)
    
    # 3. Create mass body (1kg box)
    mass = chrono.ChBody()
    mass.SetMass(1)
    mass.SetPos(chrono.ChVectorD(0, -1, 0))  # Initial position below ground
    mass.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
    system.Add(mass)
    
    # Add visualization to mass
    box_shape = chrono.ChBoxShape()
    box_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.2, 0.2)
    mass.AddVisualShape(box_shape)
    
    # 4. Create spring-damper between ground and mass
    spring = chrono.ChLinkTSDA()
    spring.Initialize(ground, mass, True, 
                      chrono.ChVectorD(0, 0, 0),  # Point on ground
                      chrono.ChVectorD(0, 0, 0))   # Point on mass
    spring.SetSpringCoefficient(100)  # 100 N/m
    spring.SetDampingCoefficient(1)   # 1 N·s/m
    spring.SetRestLength(1.0)         # Natural length = initial distance
    system.Add(spring)
    
    # Add spring visualization
    spring_vis = chrono.ChSpringShape()
    spring_vis.SetNumRenderPoints(20)
    spring_vis.SetSpringRadius(0.05)
    spring.AddVisualShape(spring_vis)
    
    # 5. Set up Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Mass-Spring-Damper System')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1, -1, 1), chrono.ChVectorD(0, -1, 0))
    vis.AddTypicalLights()
    
    # 6. Simulation loop
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()