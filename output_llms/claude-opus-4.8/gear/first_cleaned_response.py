import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math




sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


mat = chrono.ChContactMaterialNSC()




radA = 1.5          
wheel_thickness = 0.5
radB = 1.0          
interaxis = radA + radB   







truss = chrono.ChBodyEasyBox(interaxis + 4.2, 1.5, 0.5,   
                             1000,    
                             True,    
                             False,   
                             mat)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
sys.Add(truss)


gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                  radA,            
                                  wheel_thickness, 
                                  1000,            
                                  True, False, mat)
gearA.SetPos(chrono.ChVector3d(0, 0, 0))

gearA.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
gearA.GetVisualShape(0).SetColor(chrono.ChColor(1.0, 0.4, 0.4))
sys.Add(gearA)


link_truss_gearA = chrono.ChLinkLockLock()
link_truss_gearA.Initialize(gearA, truss,
                            chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(link_truss_gearA)


bar = chrono.ChBodyEasyBox(interaxis + 0.4, 0.4, 0.4,
                           1000, True, False, mat)

bar.SetPos(chrono.ChVector3d(interaxis / 2.0, 0, -1.0))
bar.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.9))
sys.Add(bar)


link_bar = chrono.ChLinkLockRevolute()
link_bar.Initialize(bar, truss,
                    chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(link_bar)


gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                  radB,
                                  wheel_thickness,
                                  1000, True, False, mat)
gearB.SetPos(chrono.ChVector3d(interaxis, 0, 0))
gearB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
gearB.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.9, 0.4))
sys.Add(gearB)


link_planet = chrono.ChLinkLockRevolute()
link_planet.Initialize(gearB, bar,
                        chrono.ChFramed(chrono.ChVector3d(interaxis, 0, 0)))
sys.Add(link_planet)








gear_constraint = chrono.ChLinkLockGear()
gear_constraint.Initialize(gearA, gearB,
                           chrono.ChFramed())
gear_constraint.SetFrameShaft1(
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                    chrono.QuatFromAngleX(-chrono.CH_PI_2)))
gear_constraint.SetFrameShaft2(
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                    chrono.QuatFromAngleX(-chrono.CH_PI_2)))
gear_constraint.SetTransmissionRatio(radA / radB)
gear_constraint.SetEpicyclic(True)   
sys.AddLink(gear_constraint)



motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                 chrono.QuatFromAngleX(chrono.CH_PI_2)))
motor.SetSpeedFunction(chrono.ChFunctionConst(0.5))   
sys.AddLink(motor)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1000, 800)
vis.SetWindowTitle('Epicyclic Gears Demo')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 4, -8),
              chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 8, -5),
                       chrono.ChVector3d(0, 0, 0),
                       12, 2, 12, 55)





sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    chronoirr.drawSegment(vis,
                          chrono.ChVector3d(0, 0, 0),
                          chrono.ChVector3d(interaxis, 0, 0),
                          chrono.ChColor(0, 1, 0), True)
    vis.EndScene()

    sys.DoStepDynamics(1e-3)