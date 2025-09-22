import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mklsolver
import pychrono.irrlicht as chronoirr
import os




class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()

    
    def Get_y(self, t):
        
        if t > 0.5:
            return chrono.CH_PI                     
        else:
            
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * t / 0.3)) / 2.0





sys = chrono.ChSystemSMC()           




L = 1.2
H = 0.4
K = 0.07

vA = chrono.ChVectorD(0,   0, 0)
vC = chrono.ChVectorD(L,   0, 0)
vB = chrono.ChVectorD(L,  -H, 0)
vG = chrono.ChVectorD(L-K,-H, 0)
vd = chrono.ChVectorD(0, 0, 1e-4)          




body_truss = chrono.ChBody()
body_truss.SetBodyFixed(True)
sys.Add(body_truss)


box_truss = chrono.ChVisualShapeBox(chrono.ChVectorD(0.015, 0.125, 0.075))
body_truss.AddVisualShape(box_truss,
                          chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))




body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.Add(body_crank)

box_crank = chrono.ChVisualShapeBox(chrono.ChVectorD(K*0.5, 0.025, 0.015))
body_crank.AddVisualShape(box_crank)

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank,
                 chrono.ChFrameD(vG, chrono.QUNIT))   
motor.SetSpeedFunction(ChFunctionMyFun())
sys.Add(motor)




mesh = fea.ChMesh()


beam_wy, beam_wz = 0.12, 0.15     

iner_coss = fea.ChInertiaCosseratSimple()
iner_coss.SetAsRectangularSection(beam_wy, beam_wz, 2700)      

elastic_coss = fea.ChElasticityCosseratSimple()
elastic_coss.SetYoungModulus(72e9)
elastic_coss.SetShearModulusFromPoisson(0.35)
elastic_coss.SetAsRectangularSection(beam_wy, beam_wz)

section_coss = fea.ChSectionCosserat(iner_coss, elastic_coss)
section_coss.SetDrawThickness(beam_wy * 0.5, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, section_coss, 30, vA, vC, chrono.VECT_X, 3)


builder_iga.GetLastBeamNodes()[0].SetFixed(True)


node_tip = builder_iga.GetLastBeamNodes()[65]    
node_mid = builder_iga.GetLastBeamNodes()[32]


hbeam_d = 0.05
section_vert = fea.ChBeamSectionAdvanced()
section_vert.SetDensity(2500)
section_vert.SetYoungModulus(75e9)
section_vert.SetShearModulusFromPoisson(0.25)
section_vert.SetRayleighDamping(0.0)
section_vert.SetAsCircularSection(hbeam_d)

builder_vert = fea.ChBuilderBeamEuler()
builder_vert.BuildBeam(mesh, section_vert, 10, vC + vd, vB + vd,
                       chrono.ChVectorD(1, 0, 0))      

node_top  = builder_vert.GetLastBeamNodes()[0]
node_down = builder_vert.GetLastBeamNodes()[-1]


crankbeam_d = 0.06
section_crank = fea.ChBeamSectionAdvanced()
section_crank.SetDensity(2800)
section_crank.SetYoungModulus(75e9)
section_crank.SetShearModulusFromPoisson(0.25)
section_crank.SetRayleighDamping(0.0)
section_crank.SetAsCircularSection(crankbeam_d)

builder_crank = fea.ChBuilderBeamEuler()
builder_crank.BuildBeam(mesh, section_crank, 4, vG + vd, vB + vd,
                        chrono.ChVectorD(0, 1, 0))

node_crankG = builder_crank.GetLastBeamNodes()[0]
node_crankB = builder_crank.GetLastBeamNodes()[-1]





constr_bb = chrono.ChLinkMateParallel()
constr_bb.Initialize(node_top, node_tip, False,
                     node_top.Frame(), node_top.Frame())
constr_bb.SetConstrainedCoords(True, False, True, False, False, False)
sys.Add(constr_bb)


constr_cbd = chrono.ChLinkMatePrismatic()
constr_cbd.Initialize(node_crankG, body_crank, False,
                      node_crankG.Frame(), node_crankG.Frame())
constr_cbd.SetConstrainedCoords(True, True, True, True, True, True)
sys.Add(constr_cbd)


constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False,
                     node_crankB.Frame(), node_crankB.Frame())
constr_bc.SetConstrainedCoords(True, True, False, True, True, False)
sys.Add(constr_bc)


constr_bb.AddVisualShape(chrono.ChVisualShapeSphere(0.02))
constr_bc.AddVisualShape(chrono.ChVisualShapeSphere(0.01))




mesh.SetAutomaticGravity(True)
sys.Add(mesh)




vis_beam = chrono.ChVisualShapeFEA(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
vis_beam.SetColorscaleMinMax(-400, 400)
vis_beam.SetSmoothFaces(False)
vis_beam.SetWireframe(False)
mesh.AddVisualShapeFEA(vis_beam)

vis_glyphs = chrono.ChVisualShapeFEA(mesh)
vis_glyphs.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_glyphs.SetSymbolsThickness(0.005)
vis_glyphs.SetSymbolsScale(0.01)
vis_glyphs.SetZbufferHide(True)
mesh.AddVisualShapeFEA(vis_glyphs)




mkl_solver = mklsolver.ChSolverPardisoMKL()
sys.SetSolver(mkl_solver)

stepper = chrono.ChTimestepperHHT(sys)     
stepper.SetAlpha(-0.2)
stepper.SetMaxiters(50)
stepper.SetAbsTolerances(1e-10)
sys.SetTimestepper(stepper)




vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam mechanism – corrected version')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.jpg'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 1.0, -1.5))
vis.AddTypicalLights()




time_step = 0.002
while vis.Run():
    vis.BeginScene()
    vis.Render()

    chronoirr.drawGrid(vis, 0.1, 0.1, 10, 10,
                       chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))

    vis.EndScene()
    sys.DoStepDynamics(time_step)