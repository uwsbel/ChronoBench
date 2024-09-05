import chrono as ch
import irrlight as irrl


ch.InitChSystem()


ground = ch.ChBodyEasyBox(10, 0.1, 10, 1e-3, 1e-3, 1e-3, 0, 0, 0)
ground.SetPos(ch.Vec(0, -0.05, 0))
ground.SetRot(ch.Mat33.Identity())
ground.SetBodyFixed(True)


mass = ch.ChBodyEasyBox(0.1, 0.1, 0.1, 1, 1, 1, 0, 0, 0)
mass.SetPos(ch.Vec(0, 0.05, 0))
mass.SetRot(ch.Mat33.Identity())
mass.SetBodyMass(1)


spring = ch.ChLinkTSDA()
spring.SetBodyA(ground)
spring.SetBodyB(mass)
spring.SetPosA(ch.Vec(0, -0.05, 0))
spring.SetPosB(ch.Vec(0, 0.05, 0))
spring.SetRotA(ch.Mat33.Identity())
spring.SetRotB(ch.Mat33.Identity())
spring.SetK(1000)  
spring.SetC(10)    
spring.SetDamping(0.1)  


irrl.InitIrrlicht()
irrl.SetCurrentWindow(irrl.CreateWindow(irrl.VideoMode(800, 600, 32)))
irrl.SetCurrentWindow(irrl.CreateDevice(irrl.EDT_OPENGL))


irrl.SetCameraTarget(ch.Vec(0, 0.05, 0))
irrl.SetCameraDistance(5)
irrl.SetCameraRotation(ch.Vec(0, 0, 0))


irrl.SetLight(irrl.ELIGHT0, irrl.ELIGHT0, irrl.ELIGHT0, irrl.ELIGHT0, irrl.ELIGHT0, irrl.ELIGHT0)
irrl.SetLightPosition(irrl.ELIGHT0, ch.Vec(0, 0, 0))
irrl.SetLightColor(irrl.ELIGHT0, ch.Vec(1, 1, 1))


mass.SetVisualShape(ch.ChShapeBox(0.1, 0.1, 0.1))
spring.SetVisualShape(ch.ChShapeSpring(0.1, 0.1, 0.1))


ch.RunSimulation(10)


ch.ClearChSystem()
irrl.CloseDevice()
irrl.CloseWindow()