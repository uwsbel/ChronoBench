import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fem as fem


chrono.SetChronoDataPath('../data/')


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


beam = fem.ChElementBeamIGA()
beam.SetN(10)  
beam.SetP(3)   


beam.SetYoungModulus(210e9)  
beam.SetDensity(7800)        
beam.SetCrossArea(0.01)      
beam.SetAsymmetry(0.5)       


beam.SetLength(1.0)          


system.Add(beam)


flywheel = chrono.ChBodyEasyCylinder(0.1, 0.05, 1000, True, True)
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  
system.Add(flywheel)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetMarker('END'), flywheel.GetMarker('END'))
motor.SetMotorFunction(chrono.ChFunction_Const(chrono.ChVectorD(10)))  
system.Add(motor)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Jeffcott Rotor Simulation')
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddTypicalLights()
visualization.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


fem_visualization = fem.ChVisualizationFEMmesh()
fem_visualization.SetMeshType(fem.ChVisualizationFEMmesh.E_LESS_LINES)
fem_visualization.SetSmoothFaces(True)
fem_visualization.SetWireframe(True)
fem_visualization.SetDrawContacts(True)
fem_visualization.SetDrawBeams(True)
fem_visualization.SetDrawElements(True)
fem_visualization.SetDrawNodes(True)
fem_visualization.SetColorScaleMin(0)
fem_visualization.SetColorScaleMax(1)
beam.AddVisualization(fem_visualization)


system.SetSolverType(chrono.ChSolver.MINRES)
system.SetMaxPenetrationRecoverySpeed(1.0)
system.SetTimestepperType(chrono.ChTimestepper.HHT)
system.SetLcpSolverType(chrono.ChSolver.PARDISO)
system.SetMaxIterationsLcp(100)
system.SetMaxPenetrationRecoverySpeed(0.1)
system.SetLcpSolverMaxIterations(100)


step_size = 0.01
total_time = 10.0


while visualization.Run():
    system.DoStepDynamics(step_size)
    visualization.Render()


visualization.Close()