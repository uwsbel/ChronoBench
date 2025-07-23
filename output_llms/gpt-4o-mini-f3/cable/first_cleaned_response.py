import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr












sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()



msection = fea.ChBeamSectionEulerAdvanced()


msection.SetAsRectangularSection(0.12, 0.24)


msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)  
msection.SetRayleighDamping(0.000)      


msection.SetQuadraticCurve(False)  
msection.SetQ1(0.04)                
msection.SetMass(0.0)                



ch_p1 = chrono.ChVector3d(-0.2, 0, 0)
ch_fixed_point = fea.ChNodeFEAxyz(ch_p1)
ch_fixed_point.SetMarkup(False)  
mesh.AddNode(ch_fixed_point)      


beam1 = fea.ChElementBeamANCF()
beam1.SetNodes(ch_fixed_point,  
               fea.ChNodeFEAxyzrot(chrono.ChVector3d(0.8, 0, 0)))  
beam1.SetSection(msection)  
mesh.AddElement(beam1)      


visualbeamA = chrono.ChVisualShapeFEA(beam1)
visualbeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualbeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualbeamA.SetSmoothFaces(True)            
visualbeamA.SetWireframe(False)              
beam1.AddVisualShapeFEA(visualbeamA)        






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA beams')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 0.6, 0.8))  
vis.AddTypicalLights()  





sys.Add(mesh)  





while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)