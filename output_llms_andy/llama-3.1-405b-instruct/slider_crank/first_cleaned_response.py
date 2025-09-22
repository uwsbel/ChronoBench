import pychrono.core as chrono
import pychrono.irrlicht as chronoirr


time_step = 0.001
time_end = 10


sys = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(sys, "PyChrono Crank-Slider Simulation", chronoirr.dimension2du(800, 600))


camera = application.GetSceneManager().getActiveCamera()
camera.setPosition(chrono.ChVectorD(0, 0, -2))
camera.setTarget(chrono.ChVectorD(0, 0, 0))


floor = chrono.ChBodyEasyBox(sys,  
                              100,  
                              10,  
                              1,   
                              10,  
                              True,  
                              False,  
                              0.5)  
floor.SetPos(chrono.ChVectorD(0, -5, 0))
floor.SetBodyFixed(True)
sys.Add(floor)


crankshaft = chrono.ChBodyEasyCylinder(sys,  
                                       500,  
                                       0.5,  
                                       5,   
                                       True,  
                                       True,  
                                       0.5)  
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
sys.Add(crankshaft)


connecting_rod = chrono.ChBodyEasyBox(sys,  
                                      300,  
                                      2,   
                                      0.5,  
                                      2,   
                                      True,  
                                      True,  
                                      0.5)  
connecting_rod.SetPos(chrono.ChVectorD(3, 0, 0))
sys.Add(connecting_rod)


piston = chrono.ChBodyEasyBox(sys,  
                              200,  
                              1,   
                              1,   
                              1,   
                              True,  
                              True,  
                              0.5)  
piston.SetPos(chrono.ChVectorD(6, 0, 0))
sys.Add(piston)


revolute_joint = chrono.ChLinkRevolute()
revolute_joint.Initialize(crankshaft, connecting_rod, chrono.ChVectorD(0, 0, 0))
sys.Add(revolute_joint)

prismatic_joint = chrono.ChLinkSlider()
prismatic_joint.Initialize(connecting_rod, piston, chrono.ChVectorD(1, 0, 0))
sys.Add(prismatic_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, chrono.ChVectorD(0, 0, 1), 10)
sys.Add(motor)


application.AssetBindAll()
application.AssetUpdateAll()


light = application.GetSceneManager().getLight(0)
light.SetDirection(chrono.ChVectorD(0, -1, 0))
light.SetIntensity(1)


application.SetTimestep(time_step)
application.SetTryRealtime(True)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    if application.GetSystem().GetChTime() > time_end:
        break