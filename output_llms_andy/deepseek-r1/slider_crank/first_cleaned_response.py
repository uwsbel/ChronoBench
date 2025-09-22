import pychrono as chrono
import pychrono.irrlicht as chronoirr
import os

def main():
    
    system = chrono.ChSystemNSC()
    system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
    system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

    
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    ground.SetName("Floor")
    
    
    ground_vis = chrono.ChBoxShape()
    ground_vis.GetBoxGeometry().Size = chrono.ChVectorD(0.5, 0.1, 0.5)
    ground_vis.SetColor(chrono.ChColor(0.2, 0.3, 0.8))
    ground.AddAsset(ground_vis)
    
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    ground.AddAsset(texture)
    
    system.AddBody(ground)

    
    crank = chrono.ChBody()
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    crank.SetMass(0.5)
    crank.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
    
    
    crank_shape = chrono.ChBoxShape()
    crank_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.15, 0.05, 0.05)
    crank_shape.SetColor(chrono.ChColor(0.9, 0.1, 0.1))
    crank.AddAsset(crank_shape)
    system.AddBody(crank)

    
    crank_joint = chrono.ChLinkLockRevolute()
    crank_joint.Initialize(ground, crank, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), 
                                            chrono.QUNIT))
    crank_joint.SetName("CrankJoint")
    system.AddLink(crank_joint)

    
    rod = chrono.ChBody()
    rod.SetMass(0.3)
    rod.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.001))
    
    
    rod_shape = chrono.ChBoxShape()
    rod_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.3, 0.02, 0.02)
    rod_shape.SetColor(chrono.ChColor(0.1, 0.9, 0.1))
    rod.AddAsset(rod_shape)
    system.AddBody(rod)

    
    rod_joint = chrono.ChLinkLockRevolute()
    rod_joint.Initialize(crank, rod, 
                        chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0, 0), 
                                          chrono.QUNIT))
    rod_joint.SetName("RodJoint")
    system.AddLink(rod_joint)

    
    piston = chrono.ChBody()
    piston.SetPos(chrono.ChVectorD(0.6, 0, 0))
    piston.SetMass(0.2)
    piston.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
    
    
    piston_shape = chrono.ChCylinderShape()
    piston_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -0.05, 0)
    piston_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0.05, 0)
    piston_shape.GetCylinderGeometry().rad = 0.08
    piston_shape.SetColor(chrono.ChColor(0.8, 0.8, 0.1))
    piston.AddAsset(piston_shape)
    system.AddBody(piston)

    
    piston_joint = chrono.ChLinkLockRevolute()
    piston_joint.Initialize(rod, piston, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0.6, 0, 0), 
                                             chrono.QUNIT))
    piston_joint.SetName("PistonJoint")
    system.AddLink(piston_joint)

    
    guide_joint = chrono.ChLinkLockPrismatic()
    guide_joint.Initialize(ground, piston, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0.6, 0, 0), 
                                            chrono.Q_from_AngY(chrono.CH_C_PI_2)))
    guide_joint.SetName("PistonGuide")
    system.AddLink(guide_joint)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(chrono.CH_C_PI / 2))  
    motor.SetName("CrankMotor")
    system.AddLink(motor)

    
    logo = chrono.ChBodyEasyMesh(chrono.GetChronoDataFile("models/logo.obj"), 1000, True, True)
    logo.SetPos(chrono.ChVectorD(0.8, 0.2, 0))
    logo.SetBodyFixed(True)
    system.AddBody(logo)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Crank-Slider Mechanism")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.0, -1.5, 0.5), chrono.ChVectorD(0.6, 0, 0))
    vis.AddTypicalLights()
    vis.AddLightWithShadow(chrono.ChVectorD(1.5, -2.5, 3.5), chrono.ChVectorD(0, 0, 0), 10, 2, 10, 40, 512)

    
    time_step = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == '__main__':
    main()