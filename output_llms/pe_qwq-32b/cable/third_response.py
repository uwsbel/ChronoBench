import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.boxes = []
        self.trusses = []
        
        # Create section properties for all beams
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)
        
        builder = fea.ChBuilderCableANCF()
        
        for i in range(n_chains):
            y_pos = i * 0.2  # Vertical spacing between chains
            
            # Create fixed truss body
            mtruss = chrono.ChBody()
            mtruss.SetPos(chrono.ChVectorD(0, y_pos, -0.1))
            mtruss.SetFixed(True)
            system.Add(mtruss)
            self.trusses.append(mtruss)
            
            # Create box body connected to truss
            mbox = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, False, None)
            mbox.SetPos(chrono.ChVectorD(0.5, y_pos, -0.1))
            system.Add(mbox)
            self.boxes.append(mbox)
            
            # Fix box to truss
            link_box_truss = chrono.ChLinkLock()
            link_box_truss.Initialize(mbox, mtruss, chrono.ChFrameD(mbox.GetPos()))
            system.Add(link_box_truss)
            
            # Create beam with increasing elements per chain
            num_elements = 10 + i  # Increasing number of elements
            start_point = chrono.ChVectorD(0, y_pos, -0.1)
            end_point = chrono.ChVectorD(0.5, y_pos, -0.1)
            
            builder.BuildBeam(
                mesh,
                msection_cable2,
                num_elements,
                start_point,
                end_point
            )
            
            # Apply force to first node of the beam
            first_node = builder.GetLastBeamNodes().front()
            first_node.SetForce(chrono.ChVectorD(0, -0.7, 0))
            
            # Connect beam's end node to box via hinge
            last_node = builder.GetLastBeamNodes().back()
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(last_node, mbox)
            system.Add(constraint_hinge)
    
    def PrintBodyPositions(self):
        for i in range(self.n_chains):
            box = self.boxes[i]
            print(f"Chain {i}: Box position = {box.GetPos()}")

# Initialize system and mesh
sys = chrono.ChSystemSMC()
sys.SetGravity(chrono.ChVectorD(0, -9.81, 0))  # Added gravity

mesh = fea.ChMesh()
model = Model1(sys, mesh, n_chains=6)
sys.Add(mesh)

# Add visualization for FEM mesh
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

# Initialize visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Multiple FEA Cable Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0.6, -1))
vis.AddTypicalLights()

# Set solver and timestepper
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)
solver.SetVerbose(False)

ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
    model.PrintBodyPositions()