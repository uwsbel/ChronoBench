import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


sys = chrono.ChSystemNSC()


beam = fea.ChBeamIGA()
beam.SetNodesNumber(10)  
beam.SetCubicHermite()   
beam.SetYoungModulus(2e11)  
beam.SetDensity(7850)      
beam.SetCrossSectionArea(0.01)  
beam.SetInertiaJ(1e-4)     
beam.SetInertiaI(1e-4)     
sys.Add(beam)              


flywheel = chrono.ChBodyEasyBox(sys, 0.1, 0.1, 0.1, 7850)
flywheel.SetPos(chrono.ChVectorD(0, 0, 0))  
flywheel.SetRot(chrono.Q_from_AngX(0))      
sys.Add(flywheel)                          


link = chrono.ChLinkMateGeneric()
link.Initialize(beam.GetNode(5), flywheel)  
sys.Add(link)                              


motor = chrono.ChLinkMotorRotation()
motor.Initialize(beam.GetNode(0), sys.GetGround())  
motor.SetMotorFunction(chrono.ChFunction_Const(10))  
sys.Add(motor)                                  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()


fem_vis = fea.ChFEMVisualization(beam)
fem_vis.SetFEMDataType(fea.ChFEMVisualization.E_PLOT_DISPLACEMENT)
fem_vis.SetWireframe(False)
vis.AddFEMVisualization(fem_vis)


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()