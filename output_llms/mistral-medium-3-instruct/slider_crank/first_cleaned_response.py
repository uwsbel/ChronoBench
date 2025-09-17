import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


floor_body = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor_body.SetPos(chrono.ChVectorD(0, -0.25, 0))
floor_body.SetBodyFixed(True)
floor_body.GetVisualModel().AddTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(floor_body)


crank_length = 0.5
crank_body = chrono.ChBodyEasyCylinder(0.1, crank_length, 1000, True, True)
crank_body.SetPos(chrono.ChVectorD(0, 0, 0))
crank_body.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVectorD(1, 0, 0)))
system.Add(crank_body)


rod_length = 1.0
rod_body = chrono.ChBodyEasyBox(0.1, rod_length, 0.1, 1000, True, True)
rod_body.SetPos(chrono.ChVectorD(crank_length, 0, 0))
rod_body.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI/4, chrono.ChVectorD(0, 0, 1)))
system.Add(rod_body)


piston_body = chrono.ChBodyEasyBox(0.3, 0.2, 0.3, 1000, True, True)
piston_body.SetPos(chrono.ChVectorD(crank_length + rod_length, 0, 0))
system.Add(piston_body)



crank_ground_joint = chrono.ChLinkLockRevolute()
crank_ground_joint.Initialize(floor_body, crank_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.Add(crank_ground_joint)


crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crank_body, rod_body, chrono.ChCoordsysD(chrono.ChVectorD(crank_length, 0, 0)))
system.Add(crank_rod_joint)


rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(rod_body, piston_body, chrono.ChCoordsysD(chrono.ChVectorD(crank_length + rod_length, 0, 0)))
system.Add(rod_piston_joint)


piston_ground_joint = chrono.ChLinkLockPrismatic()
piston_ground_joint.Initialize(floor_body, piston_body, chrono.ChCoordsysD(chrono.ChVectorD(crank_length + rod_length, 0, 0)))
piston_ground_joint.SetLimitSpeed(0.5)  
system.Add(piston_ground_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank_body, floor_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(1.0))  
system.Add(motor)


app = chronoirr.ChIrrApp(system, "Crank-Slider Mechanism", chronoirr.dimension2du(1280, 720))
app.AddLogo(chrono.GetChronoDataFile("logo_chronoengine_alpha.png"))
app.AddSkyBox()
app.AddTypicalLights()
app.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)
app.AssetBindAll()
app.AssetUpdateAll()


app.GetDevice().getSceneManager().addCameraSceneNodeFPS()
app.GetDevice().getSceneManager().getActiveCamera().setPosition(chrono.ChVectorD(3, 2, 3))
app.GetDevice().getSceneManager().getActiveCamera().setTarget(chrono.ChVectorD(0, 0, 0))


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()