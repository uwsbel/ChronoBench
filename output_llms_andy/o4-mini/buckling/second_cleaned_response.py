import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr




class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super(ChFunctionMyFun, self).__init__()

    def GetVal(self, x):
        
        if x > 0.4:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.4)) / 2.0





sys = chrono.ChSystemSMC()


L = 1.2        
H = 0.3        
K = 0.07       

vA = chrono.ChVectorD(0,    0, 0)
vC = chrono.ChVectorD(L,    0, 0)
vB = chrono.ChVectorD(L, -  H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0,    0, 1e-4)





body_truss = chrono.ChBody()
body_truss.SetBodyFixed(True)
sys.AddBody(body_truss)


box_truss = chrono.ChVisualShapeBox(0.03, 0.25, 0.12)

body_truss.AddVisualShape(box_truss,
                          chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0),
                                          chrono.QUNIT))





body_crank = chrono.ChBody()
body_crank.SetBodyFixed(False)
body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)


box_crank = chrono.ChVisualShapeBox(K, 0.03, 0.03)
body_crank.AddVisualShape(box_crank)





motor = chrono.ChLinkMotorRotationAngle()

motor.Initialize(body_truss,
                 body_crank,
                 chrono.ChFrameD(vG))
motor.SetAngleFunction(ChFunctionMyFun())
sys.AddLink(motor)





mesh = fea.ChMesh()


beam_wy = 0.12
beam_wz = 0.012

inertia_cos = fea.ChInertiaCosseratSimple()
inertia_cos.SetAsRectangularSection(beam_wy, beam_wz, density=2700)

elastic_cos = fea.ChElasticityCosseratSimple()
elastic_cos.SetYoungModulus(73e9)
elastic_cos.SetShearModulusFromPoisson(0.3)
elastic_cos.SetAsRectangularSection(beam_wy, beam_wz)

section1 = fea.ChBeamSectionCosserat(inertia_cos, elastic_cos)
section1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()

builder_iga.BuildBeam(mesh, section1, 32, vA, vC, chrono.VECT_Y, 3)


nodes_iga = builder_iga.GetLastBeamNodes()
nodes_iga[0].SetFixed(True)
node_tip = nodes_iga[-1]
node_mid = nodes_iga[17]



section2 = fea.ChBeamSectionEulerAdvanced()
d_vert = 0.03
section2.SetDensity(2700)
section2.SetYoungModulus(73e9)
section2.SetShearModulusFromPoisson(0.3)
section2.SetRayleighDamping(0.0)
section2.SetAsCircularSection(d_vert)

builder_vert = fea.ChBuilderBeamEuler()

builder_vert.BuildBeam(mesh, section2, 6, vC + vd, vB + vd,
                       chrono.ChVectorD(1, 0, 0))

nodes_vert = builder_vert.GetLastBeamNodes()
node_top  = nodes_vert[0]
node_down = nodes_vert[-1]



section3 = fea.ChBeamSectionEulerAdvanced()
d_crank = 0.054
section3.SetDensity(2700)
section3.SetYoungModulus(73e9)
section3.SetShearModulusFromPoisson(0.3)
section3.SetRayleighDamping(0.0)
section3.SetAsCircularSection(d_crank)

builder_crank = fea.ChBuilderBeamEuler()

builder_crank.BuildBeam(mesh, section3, 5, vG + vd, vB + vd,
                        chrono.ChVectorD(0, 1, 0))

nodes_crank = builder_crank.GetLastBeamNodes()
node_crankG = nodes_crank[0]
node_crankB = nodes_crank[-1]






constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip,
                     True,
                     node_top.GetFrame(), node_top.GetFrame())
constr_bb.SetConstrainedCoords(True, True, True, False, False, False)
sys.AddLink(constr_bb)

sphere_bb = chrono.ChVisualShapeSphere(0.012)  
constr_bb.AddVisualShape(sphere_bb)



constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank,
                      True,
                      node_crankG.GetFrame(), node_crankG.GetFrame())
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)
sys.AddLink(constr_cbd)



constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB,
                     True,
                     node_crankB.GetFrame(), node_crankB.GetFrame())
constr_bc.SetConstrainedCoords(True, True, True, True, True, False)
sys.AddLink(constr_bc)

sphere_bc = chrono.ChVisualShapeSphere(0.014)  
constr_bc.AddVisualShape(sphere_bc)





mesh.SetAutomaticGravity(False)
sys.Add(mesh)


vis_beam_mx = chrono.ChVisualShapeFEA(mesh)
vis_beam_mx.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
vis_beam_mx.SetColorscaleMinMax(-500, 500)
vis_beam_mx.SetSmoothFaces(True)
vis_beam_mx.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam_mx)


vis_beam_csy = chrono.ChVisualShapeFEA(mesh)
vis_beam_csy.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_csy.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_csy.SetSymbolsThickness(0.006)
vis_beam_csy.SetSymbolsScale(0.015)   
vis_beam_csy.SetZbufferHide(False)
mesh.AddVisualShapeFEA(vis_beam_csy)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()

vis.AddCamera(chrono.ChVectorD(0.0, 0.7, -1.2))
vis.AddTypicalLights()





pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)





while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    chronoirr.drawGrid(vis,
                       0.05, 0.05, 20, 20,
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0),
                                         chrono.QUNIT))

    vis.EndScene()
    sys.DoStepDynamics(0.001)