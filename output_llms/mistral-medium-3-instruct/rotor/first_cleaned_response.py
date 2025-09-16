import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import numpy as np


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  



length = 1.0  
radius = 0.05  
density = 7800  
E = 2.1e11  
nu = 0.3  
G = E / (2 * (1 + nu))  


beam = fea.ChBeamIGA()
beam.SetDensity(density)
beam.SetYoungModulus(E)
beam.SetShearModulus(G)
beam.SetPoissonRatio(nu)


beam.SetBeamGeometry(fea.ChBeamIGA.Geometry.CYLINDER)
beam.SetBeamDimensions(radius, radius)


beam.SetNumElements(10)
beam.SetNumLayers(1)  
beam.SetNumPoints(11)  


beam.SetupInitial(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                 chrono.ChVectorD(length, 0, 0))
system.Add(beam)


flywheel = chrono.ChBody()
flywheel.SetMass(10)  
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  
flywheel.SetPos(chrono.ChVectorD(length/2, 0, 0))
system.Add(flywheel)


link_flywheel = chrono.ChLinkLockLock()
link_flywheel.Initialize(beam.GetNode(5), flywheel)
system.Add(link_flywheel)


motor = chrono.ChBody()
motor.SetMass(1)
motor.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
motor.SetPos(chrono.ChVectorD(0, 0, 0))
motor.SetBodyFixed(True)  
system.Add(motor)


link_motor = chrono.ChLinkLockRevolute()
link_motor.Initialize(beam.GetNode(0), motor)
system.Add(link_motor)


rot_speed = chrono.ChFunction_Const(10)  
motor_link = chrono.ChLinkMotorRotationSpeed()
motor_link.Initialize(link_motor, rot_speed)
system.Add(motor_link)



vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 3, 1), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)


beam_vis = fea.ChVisualizationFEAmesh(beam)
beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODES)
beam_vis.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_vis.SetFEMscaling(0.01)
beam_vis.SetSymbolsThickness(0.006)
beam_vis.SetWireframe(True)
beam_vis.SetWireframeColor(chrono.ChColor(0.2, 0.2, 0.8))
beam_vis.SetWireframeRender(True)
beam_vis.SetSurfaceRender(True)
beam_vis.SetSurfaceColor(chrono.ChColor(0.8, 0.8, 0.2))
beam.GetMesh().AddAsset(beam_vis)


flywheel_vis = chrono.ChSphereShape()
flywheel_vis.GetSphereGeometry().rad = 0.1
flywheel_vis.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
flywheel.AddAsset(flywheel_vis)


system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
system.SetMaxPenetrationRecoverySpeed(1.0)
system.SetMinBounceSpeed(0.1)


time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)