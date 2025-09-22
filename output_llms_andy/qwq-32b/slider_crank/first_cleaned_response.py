import pychrono as chrono
import pychrono.irrlicht as irr

def main():
    
    sys = chrono.ChSystemNSC()

    
    vis = irr.ChIrrApp(sys, 'Crank-Slider Mechanism', irr.dimension2du(1024, 768))
    vis.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
    vis.AddAllTypicalLights()
    vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
    vis.AddTypicalSky()

    
    floor = chrono.ChBody()
    floor.SetBodyFixed(True)
    floor.SetPos(chrono.ChVectorD(0, -0.25, 0))
    sys.AddBody(floor)

    
    floor_shape = chrono.ChBoxShape()
    floor_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 0.5, 10))
    floor.AddAsset(floor_shape)
    floor_texture = chrono.ChTexture()
    floor_texture.SetTextureFilename(chrono.GetChronoDataPath() + 'bluwhite.png')
    floor_texture.SetTextureScale(2, 2)
    floor.AddAsset(floor_texture)

    
    crankshaft = chrono.ChBody()
    crankshaft.SetMass(1)
    crankshaft.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.AddBody(crankshaft)

    
    crankshaft_shape = chrono.ChCylinderShape()
    crankshaft_shape.GetCylinderGeometry().SetHeight(0.5)  
    crankshaft_shape.GetCylinderGeometry().SetRadius(0.1)
    crankshaft.AddAsset(crankshaft_shape)

    
    rev_joint = chrono.ChLinkLockRevolute()
    rev_joint.Initialize(
        floor,
        crankshaft,
        chrono.ChFrameD(
            chrono.ChVectorD(0, 0, 0),
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)  
        )
    )
    sys.AddLink(rev_joint)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(
        floor,
        crankshaft,
        chrono.ChFrameD(
            chrono.ChVectorD(0, 0, 0),
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X)
        )
    )
    sys.AddLink(motor)
    motor.SetSpeed(chrono.ChFunction.ConstantFunction(chrono.CH_C_2_PI))  

    
    connecting_rod = chrono.ChBody()
    connecting_rod.SetMass(0.5)
    connecting_rod.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
    connecting_rod.SetPos(chrono.ChVectorD(0.1 + 0.15, 0, 0))  
    sys.AddBody(connecting_rod)

    
    rod_shape = chrono.ChCylinderShape()
    rod_shape.GetCylinderGeometry().SetHeight(0.3)  
    rod_shape.GetCylinderGeometry().SetRadius(0.05)
    connecting_rod.AddAsset(rod_shape)

    
    rev_crank_rod = chrono.ChLinkRevolute()
    rev_crank_rod.Initialize(
        crankshaft,
        connecting_rod,
        chrono.ChFrameD(
            chrono.ChVectorD(0.1, 0, 0),  
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Z)  
        )
    )
    sys.AddLink(rev_crank_rod)

    
    piston = chrono.ChBody()
    piston.SetMass(2)
    piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    piston.SetPos(chrono.ChVectorD(0.4, 0, 0))
    sys.AddBody(piston)

    
    piston_shape = chrono.ChCylinderShape()
    piston_shape.GetCylinderGeometry().SetHeight(0.2)
    piston_shape.GetCylinderGeometry().SetRadius(0.15)
    piston.AddAsset(piston_shape)

    
    pris_joint = chrono.ChLinkLockPrismatic()
    pris_joint.Initialize(
        floor,
        piston,
        chrono.ChFrameD(
            chrono.ChVectorD(0.4, 0, 0),
            chrono.QUNIT  
        )
    )
    sys.AddLink(pris_joint)

    
    rev_rod_piston = chrono.ChLinkRevolute()
    rev_rod_piston.Initialize(
        connecting_rod,
        piston,
        chrono.ChFrameD(
            chrono.ChVectorD(-0.15, 0, 0),  
            chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Z)  
        )
    )
    sys.AddLink(rev_rod_piston)

    
    vis.AssetBindAll()
    vis.AssetUpdateAll()

    
    vis.SetTimestep(0.005)
    vis.SetTryRealtime(True)

    
    while vis.Run():
        sys.DoStepDynamics(0.005)

if __name__ == '__main__':
    main()