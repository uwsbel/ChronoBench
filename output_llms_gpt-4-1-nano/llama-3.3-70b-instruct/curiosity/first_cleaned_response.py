import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.mkl as chronomkl


system = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetRestitution(0.5)
contact_material.SetCompliance(0.01)
contact_material.SetDampingF(0.1)
system.SetDefaultContactMaterial(contact_material)


ground = chrono.ChBodyEasyBox(system, 10, 1, 10, 1000, True, False)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
ground.AddCollide(chrono.ChCollisionModel())
ground.GetCollisionModel().SetBox(chrono.ChVectorD(5, 0.5, 5))
ground.GetCollisionModel().SetContactMaterial(contact_material)
system.Add(ground)


rover = chrono.ChBodyEasyBox(system, 2, 1, 2, 500, True, False)
rover.SetPos(chrono.ChVectorD(0, 1, 0))
rover.AddCollide(chrono.ChCollisionModel())
rover.GetCollisionModel().SetBox(chrono.ChVectorD(1, 0.5, 1))
rover.GetCollisionModel().SetContactMaterial(contact_material)
system.Add(rover)


motor_driver = chrono.ChLinkMotorRotation()
motor_driver.Initialize(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)), 
                        chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
rover.AddLink(motor_driver)


vis = chronoirr.ChVisualSystemIrrlicht()


vis.AddCamera(chronoirr.ChCamera())
vis.GetCamera().SetPos(chrono.ChVectorD(0, 5, -10))
vis.GetCamera().SetLookAt(chrono.ChVectorD(0, 0, 0))


vis.AddLight(chronoirr.ChLightDirectional(chrono.ChVectorD(1, 1, 1), 
                                          chrono.ChVectorD(0.5, 0.5, 0.5), 
                                          100))
vis.AddLight(chronoirr.ChLightPoint(chrono.ChVectorD(0, 5, 0), 
                                    chrono.ChVectorD(1, 1, 1), 
                                    100))


vis.AddTexture(chronoirr.ChTexture("ground_texture.jpg"))
vis.AddLogo(chronoirr.ChLogo("NASA_logo.png"))


vis.Start()


while vis.Run():
    system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()