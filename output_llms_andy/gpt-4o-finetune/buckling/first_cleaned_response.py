import pychrono as chrono  
import pychrono.fea as fea  
import pychrono.irrlicht as chronoirr  
import math  


sys = chrono.ChSystemNSC()


sys.AddAsset(chrono.ChVisualSystemAsset())





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA beam buckling')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(30, 30, 30), chrono.ChVector3d(0, 0, 2))  
vis.AddTypicalLights()  


truss = chrono.ChBody()
truss.SetFixed(True)  
sys.Add(truss)  


mesh = fea.ChMesh()


beam_section = fea.ChBeamSectionEulerAdvanced()
beam_section.SetAsRectangularSection(0.1, 0.2)  
beam_section.SetYoungModulus(5e8)  
beam_section.SetShearModulus(2e8)  
beam_section.SetRayleighDamping(0.000)  
beam_section.SetSectionCentroid(0, 0.1)  
beam_section.SetShearCenter(0, 0.1)  
beam_section.SetWarpingConstant(0.0)  
beam_section.SetTorsionalRigidity(110e6)  


beam_thickness = 0.01  
mnode1 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))  
mnode2 = fea.ChNodeFEAxyzD(chrono.ChVector3d(0, beam_thickness, 0), chrono.ChVector3d(0, 0, 0))  
mnode3 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_thickness, beam_thickness, 0), chrono.ChVector3d(0, 0, 0))  
mnode4 = fea.ChNodeFEAxyzD(chrono.ChVector3d(beam_thickness, 0, 0), chrono.ChVector3d(0, 0, 0))  


mesh.AddNode(mnode1)
mesh.AddNode(mnode2)
mesh.AddNode(mnode3)
mesh.AddNode(mnode4)


melementbeam = fea.ChElementBeamEuler()
melementbeam.SetNodes(mnode1, mnode3)  
melementbeam.SetBeamSection(beam_section)  
mesh.AddElement(melementbeam)  


mnodef1 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))  
mnodef2 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, -beam_thickness))  
mnodef3 = fea.ChNodeFEAxyz(chrono.ChVector3d(beam_thickness, 0, -beam_thickness))  
mnodef4 = fea.ChNodeFEAxyz(chrono.ChVector3d(beam_thickness, 0, 0))  


mesh.AddNode(mnodef1)
mesh.AddNode(mnodef2)
mesh.AddNode(mnodef3)
mesh.AddNode(mnodef4)


melementf1 = fea.ChElementTetra_4()
melementf1.SetNodes(mnodef1, mnodef2, mnodef3, mnodef4)  
melementf1.SetMaterial(chrono.ChContinuumElastic())  
melementf1.GetMaterial().SetYoungModulus(5e8)  
melementf1.GetMaterial().SetPoissonRatio(0.3)  
mesh.AddElement(melementf1)  


melementf2 = fea.ChElementTetra_4()
melementf2.SetNodes(mnodef1, mnodef4, mnode3, mnode1)  
melementf2.SetMaterial(chrono.ChContinuumElastic())  
melementf2.GetMaterial().SetYoungModulus(5e8)  
melementf2.GetMaterial().SetPoissonRatio(0.3)  
mesh.AddElement(melementf2)  


visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_MX)
visualizebeamA.SetColorscaleMinMax(-50, 50)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)  


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamB.SetSymbolsThickness(0.006)  
visualizebeamB.SetStripThickness(0.006)  
visualizebeamB.SetStripDistribution(True)  
visualizebeamB.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)  
visualizebeamB.SetSmoothFaces(True)  
mesh.AddVisualShapeFEA(visualizebeamB)  


mtrussbeam_top = chrono.ChBody()
mtrussbeam_top.SetFixed(False)  
sys.Add(mtrussbeam_top)  


melementbeam3 = fea.ChElementBeamEuler()
melementbeam3.SetNodes(mnode3, mtrussbeam_top)  
melementbeam3.SetBeamSection(beam_section)  
mesh.AddElement(melementbeam3)  


visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetStripThickness(0.006)
visualizebeamC.SetStripDistribution(True)
visualizebeamC.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)
visualizebeamC.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamC)


mtrussbeam_h = chrono.ChBody()
mtrussbeam_h.SetFixed(True)  
sys.Add(mtrussbeam_h)


melementbeam2 = fea.ChElementBeamEuler()
melementbeam2.SetNodes(mtrussbeam_beam_top, mtrussbeam_h)  
melementbeam2.SetBeamSection(beam_section)  
mesh.AddElement(melementbeam2)


visualizebeamD = chrono.ChVisualShapeFEA(mesh)
visualizebeamD.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BEAM_MZ)
visualizebeamD.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_BEAM_MZ)
visualizebeamD.SetSymbolsThickness(0.006)
visualizebeamD.SetStripThickness(0.006)
visualizebeamD.SetStripDistribution(True)
visualizebeamD.SetSweepAxis(chrono.ChVisualShapeFEA.E_SweepAxis_BMZ)
visualizebeamD.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizebeamD)


mforcegravity = chrono.ChForceGravity()
mforcegravity.SetGravity(chrono.ChVector3d(0, -10, 0))  
sys.AddForce(mforcegravity)  


constraintherebeam = chrono.ChLinkMateGeneric()
constraintherebeam.Initialize(melementbeam3, True, mtrussbeam_beam_top, True, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(constraintherebeam)  
constraintherebeam.SetMotionAlongLock(chrono.ChAxis_X, True)  


constraintheremotor = chrono.ChLinkMateGeneric()
constraintheremotor.Initialize(mtrussbeam_h, True, mtrussbeam_beam_top, True, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
sys.Add(constraintheremotor)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_X, True)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_Y, True)
constraintheremotor.SetMotionAlongLock(chrono.ChAxis_Z, True)
constraintheremotor.SetMotionRotAlongLock(chrono.ChAxis_X, True)


mymotor = chrono.ChLinkMotorRotationTranslation()
mymotor.Initialize(mtrussbeam_h, mtrussbeam_beam_top, chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChFramed(chrono.ChVector3d(0, 0, 0.1), chrono.QUNIT))
sys.Add(mymotor)  


class MyCustomChFunction(chrono.ChFunction):
    def __init__(self):
        chrono.ChFunction.__init__(self)

    def GetVal(self, x):
        omega = 2
        return (180 / math.pi) * (1 - chrono.exp(-omega * x))  


mymotor.SetMotorFunction(MyCustomChFunction())


load_container = chrono.ChLoadContainer()
sys.Add(load_container)


myloadD = chrono.ChLoaderRayleigh(0.0, 0, 0)  
load_container.Add(myloadD)  


sys.Add(mesh)


sys.SetSolverType(chrono.ChSolver.Type_ANITESCU)
msolver = sys.GetSolver()
if msolver.GetType() == chrono.ChSolver.Type_ANITESCU:
    msolver.SetDiagonalPreconditioning(True)  
    msolver.SetSparseStep(ChronoSolverSparse.DYNAMIC_SPARSE_STEP_CHOLESKY)
    msolver.SetMaxIterations(80)  
    msolver.SetTolerance(1e-10)  
    msolver.EnableWarmStart(True)  


timestepper = chrono.ChTimestepperHHT()
sys.SetTimestepper(timestepper)


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.001)