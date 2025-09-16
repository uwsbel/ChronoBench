import pychrono as chrono
import pychrono.irrlicht as irr
import math

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    
    visualizer = irr.ChVisualSystemIrrlicht()
    visualizer.SetWindowSize(1024, 768)
    visualizer.SetWindowTitle('Crank-Slider Mechanism')
    visualizer.SetSymbolscale(0.01)
    visualizer.SetShadows(True)
    visualizer.SetLightDirection(chrono.ChVectorD(0.7, -0.2, -0.3))
    visualizer.SetCameraPosition(chrono.ChVectorD(0.5, 1.5, 2))
    visualizer.SetCameraRotation(chrono.Q_from_Euler_123(chrono.ChVectorD(-1.2, 0, 0)))
    system.SetVisualSystem(visualizer)

    
    billboard = chrono.ChBillboard()
    billboard.SetTexture(chrono.GetChronoDataFile('logo_chrono.png'))
    billboard.SetSize(0.2)
    billboard.SetPosition(chrono.ChVectorD(-2, 1, 2))
    visualizer.AddBillboard(billboard)

    
    floor = chrono.ChBodyEasyBox(5, 0.1, 5, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
    floor.SetBodyFixed(True)
    system.Add(floor)

    
    texture_floor = chrono.ChTexture()
    texture_floor.SetTextureFilename(chrono.GetChronoDataFile('textures/wood.jpg'))
    floor.AddAsset(texture_floor)

    
    crank_length = 0.3
    crank_radius = 0.1
    crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 1000, True, True)
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(crank)
    crank.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.8, 0.8)))

    
    rev_crank = chrono.ChLinkLockRevolute()
    rev_crank.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    system.AddLink(rev_crank)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, floor, chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))
    motor.SetSpeedFunction(lambda: 2.0)  
    system.Add(motor)

    
    rod_length = math.sqrt(0.5**2 + 0.15**2)  
    rod_radius = 0.05
    rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 1000, True, True)
    rod.SetPos(chrono.ChVectorD(0.25, 0.075, 0))  
    system.Add(rod)
    rod.AddAsset(chrono.ChColorAsset(chrono.ChColor(0, 0, 1)))

    
    rev_crank_rod = chrono.ChLinkLockRevolute()
    rev_crank_rod.Initialize(crank, rod, chrono.ChFrameD(chrono.ChVectorD(0, 0.15, 0), chrono.QUNIT))
    system.AddLink(rev_crank_rod)

    
    piston_radius = 0.15
    piston_length = 0.2
    piston = chrono.ChBodyEasyCylinder(piston_radius, piston_length, 1000, True, True)
    piston.SetPos(chrono.ChVectorD(0.5, 0, 0))
    system.Add(piston)
    piston.AddAsset(chrono.ChColorAsset(chrono.ChColor(1, 0, 0)))

    
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT), chrono.VECT_X)
    system.AddLink(prismatic)

    
    rev_rod_piston = chrono.ChLinkLockRevolute()
    rev_rod_piston.Initialize(rod, piston, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0), chrono.QUNIT))
    system.AddLink(rev_rod_piston)

    
    visualizer.AddLogo(chrono.GetChronoDataFile('logo_chrono.png'))
    visualizer.SetCameraPosition(chrono.ChVectorD(0.5, 1.5, 2))
    visualizer.SetCameraRotation(chrono.Q_from_Euler_123(chrono.ChVectorD(-1.2, 0, 0)))

    while visualizer.Run():
        system.DoStepDynamics(1e-3)
        visualizer.BeginScene()
        visualizer.DrawAll()
        visualizer.EndScene()

if __name__ == '__main__':
    main()