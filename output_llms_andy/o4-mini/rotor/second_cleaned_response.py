import math
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


CH_PI = math.pi


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800.0)
minertia.SetArea(  CH_PI * (beam_ro**2 - beam_ri**2) )
minertia.SetIyy( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
minertia.SetIzz( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
melasticity.SetIzz( (CH_PI/4.0) * (beam_ro**4 - beam_ri**4) )
melasticity.SetJ(   (CH_PI/2.0) * (beam_ro**4 - beam_ri**4) )

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


nodes = builder.GetLastBeamNodes()
n_nodes = nodes.size()


mid_idx  = int(n_nodes/2)
node_mid = nodes[mid_idx]





mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.30, 0.1, 7800.0)

mbodyflywheel.SetFrame_REF_to_abs(
    chrono.ChFrameD(
        node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
        chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Z)
    )
)
sys.Add(mbodyflywheel)


fix = chrono.ChLinkMateFix()
fix.Initialize(node_mid, mbodyflywheel)
sys.Add(fix)




truss = chrono.ChBody()
truss.SetBodyFixed(True)
sys.Add(truss)


bearingL = chrono.ChLinkMateGeneric(False, True, True,  False, True, True)
bearingL.Initialize(
    nodes[0], 
    truss,
    chrono.ChFrameD(nodes[0].GetPos())
)
sys.Add(bearingL)


bearingR = chrono.ChLinkMateGeneric(False, True, True,  False, True, True)
bearingR.Initialize(
    nodes[n_nodes-1],
    truss,
    chrono.ChFrameD(nodes[n_nodes-1].GetPos())
)
sys.Add(bearingR)




rotmotor = chrono.ChLinkMotorRotationSpeed()
rotmotor.Initialize(
    nodes[0],    
    truss,       
    chrono.ChFrameD(
        nodes[0].GetPos(),
        chrono.QuatFromAngleAxis(CH_PI/2.0, chrono.VECT_Y)
    )
)
sys.Add(rotmotor)


f_ramp = chrono.ChFunctionSine(60.0, 0.1)
rotmotor.SetMotorFunction(f_ramp)




vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
vis_beam.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_beam)

vis_nodes = chrono.ChVisualShapeFEA(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_nodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_nodes.SetSymbolsThickness(0.006)
vis_nodes.SetSymbolsScale(0.01)
vis_nodes.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_nodes)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('IGA Beam Jeffcott Rotor')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(0, 2, 8),            
    chrono.ChVector3d(beam_L/2.0, 0, 0)
)
vis.AddTypicalLights()




msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


sys.DoStaticLinear()


step_size = 0.002
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(step_size)