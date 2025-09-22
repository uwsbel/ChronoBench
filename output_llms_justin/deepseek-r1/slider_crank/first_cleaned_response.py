import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, -1, 0))
    
    
    ground_shape = chrono.ChBoxShape()
    ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
    ground.AddVisualShape(ground_shape)
    
    
    ground_texture = chrono.ChTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddVisualShape(ground_shape).SetTexture(ground_texture)
    system.Add(ground)

    
    crank = chrono.ChBody()
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    crank_shape = chrono.ChBoxShape(chrono.ChVectorD(0.4, 0.1, 0.1))
    crank_shape.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
    crank.AddVisualShape(crank_shape)
    system.Add(crank)

    
    rod = chrono.ChBody()
    rod.SetPos(chrono.ChVectorD(1.5, 0, 0))
    rod_shape = chrono.ChBoxShape(chrono.ChVectorD(1.0, 0.1, 0.1))
    rod_shape.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
    rod.AddVisualShape(rod_shape)
    system.Add(rod)

    
    piston = chrono.ChBody()
    piston.SetPos(chrono.ChVectorD(2.5, 0, 0))
    piston_shape = chrono.ChBoxShape(chrono.ChVectorD(0.3, 0.3, 0.3))
    piston_shape.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
    piston.AddVisualShape(piston_shape)
    system.Add(piston)

    
    
    motor_link = chrono.ChLinkMotorRotationSpeed()
    motor_link.Initialize(ground, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor_func = chrono.ChFunction_Ramp(0, 1.0)  
    motor_link.SetSpeedFunction(motor_func)
    system.Add(motor_link)

    
    joint_crank_rod = chrono.ChLinkRevolute()
    joint_crank_rod.Initialize(crank, rod, 
                              chrono.ChCoordsysD(chrono.ChVectorD(0.8, 0, 0)))
    system.Add(joint_crank_rod)

    
    joint_rod_piston = chrono.ChLinkRevolute()
    joint_rod_piston.Initialize(rod, piston, 
                               chrono.ChCoordsysD(chrono.ChVectorD(2.0, 0, 0)))
    system.Add(joint_rod_piston)

    
    prismatic_link = chrono.ChLinkLockPrismatic()
    prismatic_link.Initialize(ground, piston, 
                             chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                             chrono.Q_ROTATE_Z_TO_X))
    system.Add(prismatic_link)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Crank-Slider Mechanism')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(3, 1, 4), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.RenderFrame(chrono.ChFrameD(), 2.0)  
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()