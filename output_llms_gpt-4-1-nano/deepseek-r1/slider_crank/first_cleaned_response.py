import pychrono as chrono
import pychrono.irrlicht as irrlicht
import math

def main():
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    
    
    
    floor = chrono.ChBodyEasyBox(5, 0.2, 5, 1000, True, True)
    floor.SetName("Floor")
    floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
    floor.SetBodyFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    system.Add(floor)

    
    crank = chrono.ChBodyEasyCylinder(0.1, 0.2, 1000)  
    crank.SetName("Crankshaft")
    crank.SetPos(chrono.ChVectorD(0, 0, 0))
    crank.SetRot(chrono.QUNIT)
    crank.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.6, 0.6))
    system.Add(crank)

    
    rod = chrono.ChBodyEasyBox(0.5, 0.05, 0.05, 1000)  
    rod.SetName("Connecting Rod")
    rod.SetPos(chrono.ChVectorD(0.45, 0, 0))
    rod.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.8))
    system.Add(rod)

    
    piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    piston.SetName("Piston")
    piston.SetPos(chrono.ChVectorD(0.7, 0, 0))
    piston.GetVisualShape(0).SetColor(chrono.ChColor(0.8, 0.2, 0.2))
    system.Add(piston)

    
    
    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    motor.SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))  
    system.Add(motor)

    
    rev_crank_rod = chrono.ChLinkLockRevolute()
    rev_crank_rod.Initialize(crank, rod, 
                           chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.QUNIT))
    system.Add(rev_crank_rod)

    
    rev_rod_piston = chrono.ChLinkLockRevolute()
    rev_rod_piston.Initialize(rod, piston, 
                            chrono.ChCoordsysD(chrono.ChVectorD(0.7, 0, 0), chrono.QUNIT))
    system.Add(rev_rod_piston)

    
    prism_piston = chrono.ChLinkLockPrismatic()
    prism_piston.Initialize(floor, piston, 
                          chrono.ChCoordsysD(chrono.ChVectorD(0.7, 0, 0), chrono.QUNIT))
    system.Add(prism_piston)

    
    
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Crank-Slider Mechanism Demo")
    vis.Initialize()

    
    vis.AddCamera(chrono.ChVectorD(2, 1, 3), chrono.ChVectorD(0, 0, 0))
    vis.AddLight(chrono.ChVectorD(2, 4, 2), 10, chrono.ChColor(1, 1, 1))
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))

    
    
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.RenderGrid(0.5, 0.5, 12, 12, 
                      chrono.ChCoordsysD(chrono.ChVectorD(0, -0.2, 0), chrono.QUNIT),
                      chrono.ChColor(0.3, 0.3, 0.3))
        vis.EndScene()
        system.DoStepDynamics(1e-3)

if __name__ == "__main__":
    main()