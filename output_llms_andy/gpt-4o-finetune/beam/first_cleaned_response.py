import pychrono as chrono                 
import pychrono.fea as fea               
import pychrono.pardisomkl as mkl       
import pychrono.irrlicht as chronoirr    
import math                             


sys = chrono.ChSystemSMC()


truss = chrono.ChBody()
truss.SetFixed(True)  
sys.Add(truss)  


mesh = fea.ChMesh()


msection = fea.ChBeamSectionEulerAdvanced()  

msection.SetAsCircularSection(0.015)  
msection.SetYoungModulus(0.01e9)  
msection.SetRayleighDamping(0.000)  
msection.SetShearModulus(0.01e9*0.3)  
msection.SetSectionStrainForm(2)  



node_1 = fea.ChNodeFEAxyz(chrono.ChVector3d(0,0,0))
node_2 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.2,0,0))
node_3 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.4,0,0))
node_4 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.6,0,0))
node_5 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.4,-0.3,0))
node_6 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.6,-0.3,0))
node_7 = fea.ChNodeFEAxyz(chrono.ChVector3d(0.8,-0.6,0))

mesh.AddNode(node_1)
mesh.AddNode(node_2)
mesh.AddNode(node_3)
mesh.AddNode(node_4)
mesh.AddNode(node_5)
mesh.AddNode(node_6)
mesh.AddNode(node_7)


belement_1 = fea.ChElementBeamEuler()
belement_1.SetNodes(node_1,node_2)
belement_1.AddSection(msection)

belement_2 = fea.ChElementBeamEuler()
belement_2.SetNodes(node_2,node_3)
belement_2.AddSection(msection)

belement_3 = fea.ChElementBeamEuler()
belement_3.SetNodes(node_3,node_4)
belement_3.AddSection(msection)

belement_4 = fea.ChElementBeamEuler()
belement_4.SetNodes(node_3,node_5)
belement_4.AddSection(msection)

belement_5 = fea.ChElementBeamEuler()
belement_5.SetNodes(node_4,node_6)
belement_5.AddSection(msection)

belement_6 = fea.ChElementBeamEuler()
belement_6.SetNodes(node_5,node_6)
belement_6.AddSection(msection)

belement_7 = fea.ChElementBeamEuler()
belement_7.SetNodes(node_6,node_7)
belement_7.AddSection(msection)


mesh.AddElement(belement_1)
mesh.AddElement(belement_2)
mesh.AddElement(belement_3)
mesh.AddElement(belement_4)
mesh.AddElement(belement_5)
mesh.AddElement(belement_6)
mesh.AddElement(belement_7)



node_1.SetFixed(True)
node_7.SetFixed(True)

load_container = fea.ChLoadContainer()
sys.Add(load_container)  



force = fea.ChLoadNodeForce()
force.SetForce(chrono.ChVector3d(0,1.0,0))  
force.SetPointAtAbs(chrono.ChVector3d(0.4,-0.15,0))  
load_container.Add(force)  
force.Loadable.AppendFootNode(node_5)  



mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
mvisualizebeamA.SetColorscaleMinMax(-0.4,0.4)  
mvisualizebeamA.SetSmoothFaces(True)  
mvisualizebeamA.SetDrawBeams(True)  
mesh.AddVisualShapeFEA(mvisualizebeamA)  
    


mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
mvisualizebeamC.SetSymbolsThickness(0.006)  
mvisualizebeamC.SetSymbolsScale(0.01)  
mvisualizebeamC.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(mvisualizebeamC)  


sys.Add(mesh)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA beams')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0.5,0.2,1.3), chrono.ChVector3d(0.4,0.0,0.3))  
vis.AddTypicalLights()  


msolver = mkl.ChSolverPardisoMKL()  
sys.SetSolver(msolver)  


timestep = 1e-3  


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(timestep)