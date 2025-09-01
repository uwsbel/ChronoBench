import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PYCHRONO_DATA_DIR')
my_system = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile('concrete.jpg'))
ground.AddAsset(ground_texture)
my_system.Add(ground)


bar = chrono.ChBodyEasyCylinder(1, 2, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 5, 0))
bar.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0)))
my_system.Add(bar)


gear1 = chrono.ChBodyEasyCylinder(1, 0.5, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(2, 5, 0))
gear2 = chrono.ChBodyEasyCylinder(1, 0.5, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(4, 5, 0))
my_system.Add(gear1)
my_system.Add(gear2)


gear1_bar_link = chrono.ChLinkLockRevolute()
gear1_bar_link.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(2, 5, 0)))
gear2_bar_link = chrono.ChLinkLockRevolute()
gear2_bar_link.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(4, 5, 0)))
gear1_gear2_link = chrono.ChLinkLockRevolute()
gear1_gear2_link.Initialize(gear1, gear2, chrono.ChCoordsysD(chrono.ChVectorD(3, 5, 0)))
my_system.Add(gear1_bar_link)
my_system.Add(gear2_bar_link)
my_system.Add(gear1_gear2_link)


gear1_motor = chrono.ChLinkMotorRotationSpeed()
gear1_motor.Initialize(gear1, chrono.ChFrameD(chrono.ChVectorD(2, 5, 0)))
gear1_motor.SetSpeed(1)  
my_system.Add(gear1_motor)


my_application = irr.ChIrrApp(my_system, 'Epicyclic Gears', irr.dimension2du(1024, 768))
my_application.AddTypicalLights()
my_application.AddCamera(irr.vector3df(0, 10, -20))
my_application.AssetBindAll()
my_application.AssetUpdateAll()


while my_application.GetDevice().run():
    my_application.BeginScene()
    my_application.DrawAll()
    my_application.DoStep()
    my_application.EndScene()