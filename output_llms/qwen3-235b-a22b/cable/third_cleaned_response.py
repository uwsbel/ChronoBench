import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.boxes = []
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  
        
        builder = fea.ChBuilderCableANCF()
        
        for i in range(n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.system.Add(mtruss)
            
            
            y_offset = 0.2 * i
            start = chrono.ChVector3d(0, y_offset, 0)
            end = chrono.ChVector3d(0.5, y_offset, 0)
            
            
            n_elements = 10 + i
            
            
            builder.BuildBeam(
                mesh, 
                msection_cable2, 
                n_elements, 
                start, 
                end
            )
            
            
            nodes = builder.GetLastBeamNodes()
            start_node = nodes.front()
            end_node = nodes.back()
            
            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(start_node, mtruss)
            self.system.Add(constraint_hinge)
            
            
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)  
            box.SetPos(end_node.GetPos())
            box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
            self.system.Add(box)
            
            
            constraint_point = fea.ChLinkPointFrame()
            constraint_point.Initialize(end_node, box)
            self.system.Add(constraint_point)
            
            
            end_node.SetForce(chrono.ChVector3d(0, -0.7, 0))
            
            
            self.boxes.append(box)
    
    def PrintBodyPositions(self):
        for i, box in enumerate(self.boxes):
            pos = box.GetPos()
            print(f"Chain {i} box position: ({pos.x}, {pos.y}, {pos.z})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)  



visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)  


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualizebeamB.SetSymbolsThickness(0.006)  
visualizebeamB.SetSymbolsScale(0.01)  
visualizebeamB.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualizebeamB)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA cables')  
vis.Initialize()  
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  
vis.AddSkyBox()  
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))  
vis.AddTypicalLights()  


solver = chrono.ChSolverMINRES() 
if solver.GetType() == chrono.ChSolver.Type_MINRES:
    print("Using MINRES solver")
    sys.SetSolver(solver)
    solver.SetMaxIterations(200)
    solver.SetTolerance(1e-10)
    solver.EnableDiagonalPreconditioner(True)
    solver.EnableWarmStart(True)  
    solver.SetVerbose(False)


ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)


while vis.Run():
    vis.BeginScene()  
    vis.Render()  
    vis.EndScene()  
    sys.DoStepDynamics(0.01)  
    model.PrintBodyPositions()