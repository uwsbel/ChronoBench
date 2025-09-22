import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


sys = chrono.ChSystemNSC()


ground = chrono.ChBodyEasyBox(sys, 10, 1, 10, 1000, True, chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.SetCollide(True)
ground.SetFriction(0.9)
ground.SetRestitution(0.1)


rover = veh.ChWheeledVehicle(sys, 
                             chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngX(0)))


rover.AddChassis(chrono.ChBodyEasyBox(sys, 1, 0.5, 1, 100, True, chrono.ChVectorD(0, 0.25, 0)))
rover.AddWheel(chrono.ChWheel(sys, 
                              chrono.ChVectorD(-0.5, 0, 0.5), 
                              chrono.ChVectorD(0, -1, 0), 
                              0.2, 
                              0.1, 
                              100))
rover.AddWheel(chrono.ChWheel(sys, 
                              chrono.ChVectorD(0.5, 0, 0.5), 
                              chrono.ChVectorD(0, -1, 0), 
                              0.2, 
                              0.1, 
                              100))
rover.AddWheel(chrono.ChWheel(sys, 
                              chrono.ChVectorD(-0.5, 0, -0.5), 
                              chrono.ChVectorD(0, -1, 0), 
                              0.2, 
                              0.1, 
                              100))
rover.AddWheel(chrono.ChWheel(sys, 
                              chrono.ChVectorD(0.5, 0, -0.5), 
                              chrono.ChVectorD(0, -1, 0), 
                              0.2, 
                              0.1, 
                              100))


driver = veh.ChMotorDriver(sys, rover)


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
sys.SetStep(0.01)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.AddLogo(0.05, 0.05, "logo.png")
vis.AddFloor(-1)
vis.SetLogoTexture("logo.png")
vis.SetTypicalSceneFloorMesh("floor.obj")
vis.SetTypicalSceneSkyBoxMesh("skybox.obj")


vis.SetWindowSize(1024, 768)
vis.SetWindowCaption("Curiosity Rover Simulation")
vis.SetCameraDistance(10)
vis.SetCameraElevation(30)
vis.SetCameraAzimuth(60)


vis.Start()


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()


vis.Stop()