import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl           
import pychrono.irrlicht as chronoirr       




sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)                             
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -3.71, 0))  




beam_L  = 10.0            
beam_ro = 0.060           
beam_ri = 0.055           
CH_PI   = m.pi            


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(CH_PI * (beam_ro**2 - beam_ri**2))
minertia.SetIyy((CH_PI/4.0) * (beam_ro**4 - beam_ri**4))
minertia.SetIzz((CH_PI/4.0) * (beam_ro**4 - beam_ri**4))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((CH_PI/4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetIzz((CH_PI/4.0) * (beam_ro**4 - beam_ri**4))
melasticity.SetJ( (CH_PI/2.0) * (beam_ro**4 - beam_ri**4) )

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)




builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh,
                  msection,
                  20,                                        
                  chrono.ChVector3d(0, 0, 0),                
                  chrono.ChVector3d(beam_L, 0, 0),           
                  chrono.VECT_Y,
                  1)


last_nodes = builder.GetLastBeamNodes()
node_first = last_nodes[0]
node_mid   = last_nodes[len(last_nodes)//2]
node_last  = last_nodes[-1]




mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.10, 7800)  
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Z)))
sys.Add(mbodyflywheel)

joint_flywheel = chrono.ChLinkMateFix()
joint_flywheel.Initialize(node_mid, mbodyflywheel)
sys.Add(joint_flywheel)




truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(node_last, truss, chrono.ChFramed(node_last.GetPos()))
sys.Add(bearing)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(node_first,
                 truss,
                 chrono.ChFramed(node_first.GetPos(),
                                 chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Y)))
sys.Add(motor)


f_ramp = chrono.ChFunctionSine(60, 0.1)       
motor.SetMotorFunction(f_ramp)




vis_beam_surface = chrono.ChVisualShapeFEA(mesh)
vis_beam_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam_surface.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam_surface)

vis_beam_cs = chrono.ChVisualShapeFEA(mesh)
vis_beam_cs.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_cs.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_cs.SetSymbolsThickness(0.006)
vis_beam_cs.SetSymbolsScale(0.01)
vis_beam_cs.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_cs)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott rotor – IGA beam (modified)')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2, 8), chrono.ChVector3d(beam_L/2, 0, 0))   
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)

sys.DoStaticLinear()      

time_step = 0.002
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)