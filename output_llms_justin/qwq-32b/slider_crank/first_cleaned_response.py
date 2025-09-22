import pychrono as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


visualiz = irr.ChIrrApp(system, 'Crank-Slider Mechanism', irr.dimension2du(800,600))
visualiz.AddTypicalSky()
visualiz.AddTypicalLogo()
visualiz.AddTypicalLights()


camera_location = chrono.ChVectorD(3, 3, 3)
camera_target = chrono.ChVectorD(0, 0, 0)
visualiz.SetCamera(camera_location, camera_target, chrono.ChVectorD(0, 1, 0))


floor = chrono.ChBodyEasyBox(5, 0.1, 5, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
floor.SetBodyFixed(True)
system.Add(floor)


texture_floor = irr.ChTexture()
texture_floor.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
floor.GetAssets().append(texture_floor)


crank = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True)
crank.SetPos(chrono.ChVectorD(0, 0.25, 0))
system.Add(crank)


rev_crank_floor = chrono.ChLinkLockRevolute()
rev_crank_floor.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0.35, 0), chrono.QUNIT))
system.AddLink(rev_crank_floor)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
motor.SetSpeed(chrono.ChFunction_Const(chrono.ChVariableDouble(chrono.CH_C_PI)))  
system.Add(motor)


rod = chrono.ChBodyEasyCylinder(0.05, 1.0, 1000, True, True)
rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
system.Add(rod)


rev_rod_crank = chrono.ChLinkLockRevolute()
rev_rod_crank.Initialize(crank, rod, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0), chrono.QUNIT))
system.AddLink(rev_rod_crank)


piston = chrono.ChBodyEasyCylinder(0.1, 0.3, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1.1, 0.25, 0))
system.Add(piston)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(floor, piston, chrono.ChFrameD(chrono.ChVectorD(0, 0.35, 0), chrono.QUNIT), chrono.ChVectorD(1, 0, 0))
system.AddLink(prismatic)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(rod, piston, 
                        chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 0), chrono.QUNIT),
                        chrono.ChFrameD(chrono.ChVectorD(0.15, 0, 0), chrono.QUNIT))
system.AddLink(rev_rod_piston)


crank.GetVisualShape(0).SetColor(chrono.ChColor(0,0,1))
rod.GetVisualShape(0).SetColor(chrono.ChColor(1,0,0))
piston.GetVisualShape(0).SetColor(chrono.ChColor(0,1,0))


billboard = irr.ChBillboard()
billboard.SetTexture(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
billboard.SetSize(0.5)
billboard.SetPos(chrono.ChVectorD(0, 1, 0))
visualiz.AddBillboard(billboard)


visualiz.SetTimestep(0.005)
visualiz.SetTryRealtime(True)


visualiz.MainLoop()

import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    visualiz = irr.ChIrrApp(system, 'Crank-Slider Mechanism', irr.dimension2du(800, 600))
    visualiz.AddTypicalSky()
    visualiz.AddTypicalLogo()
    visualiz.AddTypicalLights()

    
    camera_location = chrono.ChVectorD(3, 3, 3)
    camera_target = chrono.ChVectorD(0, 0, 0)
    visualiz.SetCamera(camera_location, camera_target, chrono.ChVectorD(0, 1, 0))

    
    floor = chrono.ChBodyEasyBox(5, 0.1, 5, 1000, True, True)
    floor.SetPos(chrono.ChVectorD(0, -0.1, 0))
    floor.SetBodyFixed(True)
    system.Add(floor)

    
    texture_floor = irr.ChTexture()
    texture_floor.SetTextureFilename(chrono.GetChronoDataPath() + 'textures/concrete.jpg')
    floor.GetAssets().append(texture_floor)

    
    crank = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True)
    crank.SetPos(chrono.ChVectorD(0, 0.25, 0))
    system.Add(crank)

    
    rev_crank_floor = chrono.ChLinkLockRevolute()
    rev_crank_floor.Initialize(floor, crank, chrono.ChFrameD(chrono.ChVectorD(0, 0.35, 0), chrono.QUNIT))
    system.AddLink(rev_crank_floor)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    motor.SetSpeed(chrono.ChFunction_Const(chrono.ChVariableDouble(chrono.CH_C_PI)))  
    system.Add(motor)

    
    rod = chrono.ChBodyEasyCylinder(0.05, 1.0, 1000, True, True)
    rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
    system.Add(rod)

    
    rev_rod_crank = chrono.ChLinkLockRevolute()
    rev_rod_crank.Initialize(crank, rod, chrono.ChFrameD(chrono.ChVectorD(0.25, 0, 0), chrono.QUNIT))
    system.AddLink(rev_rod_crank)

    
    piston = chrono.ChBodyEasyCylinder(0.1, 0.3, 1000, True, True)
    piston.SetPos(chrono.ChVectorD(1.1, 0.25, 0))
    system.Add(piston)

    
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(floor, piston, chrono.ChFrameD(chrono.ChVectorD(0, 0.35, 0), chrono.QUNIT), chrono.ChVectorD(1, 0, 0))
    system.AddLink(prismatic)

    
    rev_rod_piston = chrono.ChLinkLockRevolute()
    rev_rod_piston.Initialize(
        rod, piston,
        chrono.ChFrameD(chrono.ChVectorD(-0.5, 0, 0), chrono.QUNIT),
        chrono.ChFrameD(chrono.ChVectorD(0.15, 0, 0), chrono.QUNIT)
    )
    system.AddLink(rev_rod_piston)

    
    crank.GetVisualShape(0).SetColor(chrono.ChColor(0, 0, 1))  
    rod.GetVisualShape(0).SetColor(chrono.ChColor(1, 0, 0))    
    piston.GetVisualShape(0).SetColor(chrono.ChColor(0, 1, 0)) 

    
    billboard = irr.ChBillboard()
    billboard.SetTexture(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    billboard.SetSize(0.5)
    billboard.SetPos(chrono.ChVectorD(0, 1, 0))
    visualiz.AddBillboard(billboard)

    
    visualiz.SetTimestep(0.005)
    visualiz.SetTryRealtime(True)

    
    visualiz.MainLoop()

if __name__ == '__main__':
    main()