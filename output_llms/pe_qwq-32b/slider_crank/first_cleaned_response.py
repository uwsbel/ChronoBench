import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

def main():
    
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Crank-Slider Mechanism Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetShadows(True)
    vis.SetLightIntensity(0.7)
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.SetCamera(chrono.ChVectorD(1.5, 1.5, 2.0))
    vis.SetCameraRotation(chrono.Q_from_AngAxis(-math.pi/4, chrono.ChVectorD.Y()))
    vis.AttachSystem(sys)
    vis.Initialize()
    vis.AddLightWithShadow(chrono.ChVectorD(2,2,2), chrono.ChVectorD(0,0,0), 5, 100, 1024)

    
    floor_length, floor_width, floor_depth = 2.0, 0.1, 2.0
    floor = chrono.ChBodyEasyBox(floor_length, floor_width, floor_depth, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, -floor_width/2, 0))
    floor.SetBodyFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)

    
    crank_radius = 0.02
    crank_length = 0.2
    crank_mass = 1.0
    crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 1000)
    crank.SetPos(chrono.ChVectorD(crank_length/2, 0, 0))  
    crank.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    sys.Add(crank)

    
    rev_joint = chrono.ChLinkLockRevolute()
    rev_joint.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
    sys.Add(rev_joint)

    
    rod_radius = 0.01
    rod_length = 0.5
    rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 1000)
    rod.SetPos(chrono.ChVectorD(crank_length + rod_length/2, 0, 0))  
    rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    sys.Add(rod)

    
    rev_joint_rod_crank = chrono.ChLinkLockRevolute()
    rev_joint_rod_crank.Initialize(crank, rod, chrono.ChFrameD(chrono.ChVectorD(crank_length, 0, 0)))
    sys.Add(rev_joint_rod_crank)

    
    piston_radius = 0.03
    piston_length = 0.1
    piston = chrono.ChBodyEasyCylinder(piston_radius, piston_length, 1000)
    piston.SetPos(chrono.ChVectorD(crank_length + rod_length, 0, 0))  
    piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    sys.Add(piston)

    
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(floor, piston, chrono.ChFrameD(piston.GetPos()))
    prismatic.SetAxis(chrono.ChVectorD(1, 0, 0))  
    sys.Add(prismatic)

    
    rev_joint_rod_piston = chrono.ChLinkLockRevolute()
    rev_joint_rod_piston.Initialize(rod, piston, chrono.ChFrameD(piston.GetPos()))
    sys.Add(rev_joint_rod_piston)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))  
    sys.Add(motor)

    
    crank.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))  
    rod.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))    
    piston.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0)) 

    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        sys.DoStepDynamics(0.001)  

if __name__ == '__main__':
    main()