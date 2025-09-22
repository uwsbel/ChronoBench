import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 2.0  
beam_radius = 0.05  
beam_density = 7800  
beam_E = 2.1e11  
beam_G = 8.0e10  
beam_nu = 0.3  


beam = chrono.ChBeamIGA()
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_E)
beam.SetShearModulus(beam_G)
beam.SetPoissonRatio(beam_nu)
beam.SetRadius(beam_radius)
beam.SetLength(beam_length)


beam.SetNumElements(10)


system.Add(beam)


flywheel = chrono.ChBodyEasyCylinder(beam_radius*2, 0.1, beam_density, True, True)
flywheel.SetPos(chrono.ChVectorD(0, 0, 0))  
flywheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVectorD(1, 0, 0)))
system.Add(flywheel)


link = chrono.ChLinkLockRevolute()
link.Initialize(beam, flywheel, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(link)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam, chrono.ChFrameD(chrono.ChVectorD(-beam_length/2, 0, 0)), chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
system.Add(motor)


motor_function = chrono.ChFunction_Const(10.0)  
motor.SetSpeedFunction(motor_function)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Jeffcott Rotor with IGA Beam")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(2, 4, 2), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


beam.SetVisualizationType(chrono.ChVisualizationFEAmesh())
beam.GetVisualizationModel().SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_SURFACE)
beam.GetVisualizationModel().SetFEMglyphType(chrono.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam.GetVisualizationModel().SetFEMrenderMode(chrono.ChVisualizationFEAmesh.E_RENDER_SOLID)
beam.GetVisualizationModel().SetColorscaleMinMax(-0.1, 0.1)


time_step = 1e-4
end_time = 5.0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)
    vis.Advance(time_step)

    
    current_time = system.GetChTime()
    if current_time >= end_time:
        break