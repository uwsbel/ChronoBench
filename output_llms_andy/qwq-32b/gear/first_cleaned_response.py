import pychrono as chrono
import pychrono.irrlicht as chronoirr

def main():
    
    sys = chrono.ChSystemNSC()
    sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  

    
    application = chronoirr.ChIrrApp(sys, 'Epicyclic Gears', chronoirr.dimension2du(1024,768))
    application.SetCamera(chronoirr.CameraPosition(chrono.ChVectorD(1,1,1), chrono.ChVectorD(0,0,0)))
    application.AssetBind()
    application.AssetUpdate()
    application.AddTypicalLights()
    application.AddTypicalLogo()

    
    truss = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True)
    truss.SetBodyFixed(True)
    truss.SetPos(chrono.ChVectorD(0,0,0))
    sys.Add(truss)
    truss.GetVisualShape(0).SetMaterialTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))

    
    bar_length = 0.5
    bar_radius = 0.05
    bar = chrono.ChBodyEasyCylinder(bar_length, bar_radius, 1000, True, True)
    bar.SetPos(chrono.ChVectorD(0,0,0))
    sys.Add(bar)
    bar.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.5,0.5,0.5))

    
    joint = chrono.ChLinkRevolute()
    joint.Initialize(truss, bar, chrono.ChCoordsysD(
        chrono.ChVectorD(0,0,0),
        chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0,1,0))
    ))
    sys.AddLink(joint)
    motor = joint.GetMotor()
    motor.SetVelocity(chrono.ChFunction.Constant(chrono.GetChronoSystem(), 10))  
    motor.SetTorqueMax(1e6)

    
    gear1_radius = 0.1
    gear1_teeth = 20
    gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.01, 1000, True, True)
    gear1.SetPos(chrono.ChVectorD(0.25, 0,0))  
    sys.Add(gear1)
    gear1.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.8,0,0))

    
    link_gear1 = chrono.ChLinkLockRevolute()
    link_gear1.Initialize(bar, gear1, chrono.ChFrameD(gear1.GetPos(), chrono.QUNIT))
    sys.AddLink(link_gear1)

    
    gear2_radius = 0.15
    gear2_teeth = 30
    gear2_body = chrono.ChBodyEasyCylinder(gear2_radius, 0.01, 1000, True, True)
    gear2_pos_x = 0.25 + gear1_radius + gear2_radius  
    gear2_body.SetPos(chrono.ChVectorD(gear2_pos_x, 0,0))
    sys.Add(gear2_body)
    gear2_body.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0,0.8,0))

    
    joint_gear2 = chrono.ChLinkRevolute()
    joint_gear2.Initialize(truss, gear2_body, chrono.ChCoordsysD(
        chrono.ChVectorD(gear2_pos_x,0,0),
        chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0,1,0))
    ))
    sys.AddLink(joint_gear2)

    
    gear_constr = chrono.ChLinkGear()
    gear_constr.Initialize(gear1, gear2_body, chrono.ChFrameD(), chrono.ChFrameD())
    gear_constr.SetGearRatio(gear1_teeth / gear2_teeth)  
    sys.AddLink(gear_constr)

    
    application.SetTimestep(0.001)
    application.SetTryRealtime(True)

    
    while application.GetDevice().run():
        application.BeginScene()
        application.DrawAll()
        application.DoStep()
        application.EndScene()

if __name__ == '__main__':
    main()