import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


mysim = chrono.ChSystemNSC()


chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)


beam = fea.ChBeamSectionEulerSimple()
beam.SetAsRectangular(0.01, 0.01)  
beam.SetMaterial(chrono.ChMaterial())
beam.SetYoungModulus(2e11)  
beam.SetIz(1e-5)  
beam.SetDensity(7850)  


iga_beam = fea.ChBeamIGA(beam)
iga_beam.SetControlPoints(10)  
iga_beam.SetNURBSdegree(3)  
iga_beam.SetKnots([0, 0, 0, 0, 1, 1, 1, 1])  
iga_beam.SetLength(1)  


flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 7850, True, True)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  
iga_beam.Add(flywheel)  


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(iga_beam, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeed(10)  


mysim.Add(iga_beam)
mysim.Add(motor)


app = chronoirr.ChVisualSystemIrrlicht()
app.SetWindowSize(800, 600)
app.SetWindowTitle("Jeffcott Rotor Simulation")
app.AddTypicalLights()


app.Add(iga_beam)
app.Add(flywheel)


fem_visualizer = fea.ChFEMVisualizationFEAbeam()
fem_visualizer.SetFEMmeshColor(chrono.ChColor(1, 0, 0))
fem_visualizer.SetDisplacementScaling(1)
iga_beam.AddVisualizer(fem_visualizer)


while app.Run():
    mysim.DoStepDynamics(0.01)
    app.BeginScene()
    app.DrawAll()
    app.EndScene()