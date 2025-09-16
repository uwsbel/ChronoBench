import pychrono as chrono
import pychrono.irrlicht as irr
import math

def main():
    # Create the physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

    # Visualization setup
    visualizer = irr.ChVisualSystemIrrlicht()
    visualizer.SetWindowSize(1024, 768)
    visualizer.SetWindowTitle('Crank-Slider Mechanism')
    visualizer.SetSymbolscale(0.01)
    visualizer.SetShadows(True)
    visualizer.SetLightDirection(chrono.ChVectorD(0.7, -0.2, -0.3))
    visualizer.SetCameraPosition(chrono.ChVectorD(0.5, 1.5, 2))
    visualizer.SetCameraRotation(chrono.Q_from_Euler_123(chrono.ChVectorD(-1.2, 0, 0)))
    system.SetVisualSystem(visualizer)

    # Add a Chrono logo billboard
    billboard = chrono.ChBillboard()
    billboard.SetTexture(chrono.GetChronoDataFile('logo_chrono.png'))
    billboard.SetSize(0.2)
    billboard.SetPosition(chrono.ChVectorD(-2, 1, 2))
    visualizer.AddBillboard(billboard)

    # Floor (truss)
    floor = chrono.ChBodyEasyBox(5, 0.1, 5, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, -0.05, 0))
    floor.SetBodyFixed(True)
    system.Add(floor)

    # Texture for floor
    texture_floor = chrono.ChTexture()
    texture_floor.SetTextureFilename(chrono.GetChronoDataFile('textures/wood.jpg'))
    floor.AddAsset(texture_floor)

    # Crankshaft
    crank_length = 0.3
    crank_radius = 0.1
    crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, 1000, True, True)
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(crank)
    crank.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.8, 0.8)))

    # Revolute joint for crank
    rev_crank = chrono.ChLinkLockRevolute()
    rev_crank.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    system.AddLink(rev_crank)

    # Motor for crank rotation
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, floor, chrono.ChFrameD(chrono.VNULL, chrono.QUNIT))
    motor.SetSpeedFunction(lambda: 2.0)  # 2 rad/s
    system.Add(motor)

    # Connecting rod
    rod_length = math.sqrt(0.5**2 + 0.15**2)  # Distance between crank end and piston
    rod_radius = 0.05
    rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, 1000, True, True)
    rod.SetPos(chrono.ChVectorD(0.25, 0.075, 0))  # Midpoint between crank and piston
    system.Add(rod)
    rod.AddAsset(chrono.ChColorAsset(chrono.ChColor(0, 0, 1)))

    # Revolute joint between crank and rod
    rev_crank_rod = chrono.ChLinkLockRevolute()
    rev_crank_rod.Initialize(crank, rod, chrono.ChFrameD(chrono.ChVectorD(0, 0.15, 0), chrono.QUNIT))
    system.AddLink(rev_crank_rod)

    # Piston
    piston_radius = 0.15
    piston_length = 0.2
    piston = chrono.ChBodyEasyCylinder(piston_radius, piston_length, 1000, True, True)
    piston.SetPos(chrono.ChVectorD(0.5, 0, 0))
    system.Add(piston)
    piston.AddAsset(chrono.ChColorAsset(chrono.ChColor(1, 0, 0)))

    # Prismatic joint for piston
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(floor, piston, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.QUNIT), chrono.VECT_X)
    system.AddLink(prismatic)

    # Revolute joint between rod and piston
    rev_rod_piston = chrono.ChLinkLockRevolute()
    rev_rod_piston.Initialize(rod, piston, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0), chrono.QUNIT))
    system.AddLink(rev_rod_piston)

    # Simulation loop
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