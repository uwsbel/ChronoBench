import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -0.25, 0))
floor.SetBodyFixed(True)
system.Add(floor)


crank_radius = 0.2
crank = chrono.ChBodyEasyCircle(crank_radius, 1000, True)
crank.SetPos(chrono.ChVectorD(0, 0.2, 0))
system.Add(crank)


rod_length = 1.0
rod = chrono.ChBodyEasyBox(rod_length, 0.05, 0.05, 1000, True, True)

rod.SetPos(chrono.ChVectorD(0.5, 0.2, 0))
system.Add(rod)


piston = chrono.ChBodyEasyBox(0.3, 0.3, 0.3, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1.0, 0.2, 0))
system.Add(piston)




rev_crank = chrono.ChLinkLockRevolute()
rev_crank.Initialize(crank, floor, chrono.ChCoordsysD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngX(0)))
system.Add(rev_crank)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.Q_from_AngX(0)))
motor.SetSpeed(2.0)  
system.Add(motor)


rev_rod_crank = chrono.ChLinkLockRevolute()
rev_rod_crank.Initialize(rod, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0), chrono.Q_from_AngX(0)))
system.Add(rev_rod_crank)


rev_rod_piston = chrono.ChLinkLockRevolute()
rev_rod_piston.Initialize(piston, rod, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0.2, 0), chrono.Q_from_AngX(0)))
system.Add(rev_rod_piston)


prismatic = chrono.ChLinkLockPrismatic()
prismatic.Initialize(piston, floor, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0.2, 0), chrono.Q_from_AngX(0)))
system.Add(prismatic)


prismatic.SetAxis(chrono.ChVectorD(1, 0, 0))
prismatic.SetLimit(chrono.ChFloat(0), chrono.ChFloat(2.0))
prismatic.SetLimitFunction(chrono.ChLinkLockPrismatic.Limit_Func_LowerAndUpper)


application = irr.ChIrrApp(system, 'PyChrono Crank-Slider Mechanism', irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddSkyBox()
application.AddLightWithShadow(irr.vector3df(0, 10, 10), irr.vector3df(0, 0, 0))
application.AssetBindAll()
application.AssetUpdateAll()


application.GetDevice().getSceneManager().addCameraSceneNode(0, irr.vector3df(3, 2, -5), irr.vector3df(0, 0.5, 0))
application.GetVideoDriver().setTextureCreationFlag(irr.E_TEXTURE_CREATION_FLAG_EARLY_Z, True)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    system.DoStepDynamics(1e-3)
    application.EndScene()