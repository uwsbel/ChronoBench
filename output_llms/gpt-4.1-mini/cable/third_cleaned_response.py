import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr








class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.n_chains = n_chains
        
        self.end_bodies = []

        
        
        y_spacing = 0.15

        for i in range(self.n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)      
            msection_cable2.SetYoungModulus(0.01e9) 
            msection_cable2.SetRayleighDamping(0.0001)

            
            builder = fea.ChBuilderCableANCF()

            
            n_elements = 3 + i

            
            
            start_point = chrono.ChVector3d(0, i * y_spacing, -0.1)
            end_point = chrono.ChVector3d(0.5, i * y_spacing, -0.1)

            
            builder.BuildBeam(
                self.mesh,
                msection_cable2,
                n_elements,
                start_point,
                end_point
            )

            
            
            
            
            front_node = builder.GetLastBeamNodes().front()
            force_vector = chrono.ChVector3d(0, -0.7, 0)
            front_node.SetForce(force_vector)

            
            mtruss = chrono.ChBody()
            mtruss.SetBodyFixed(True)
            self.system.Add(mtruss)

            
            constraint_hinge = fea.ChLinkNodeFrame()
            back_node = builder.GetLastBeamNodes().back()
            
            constraint_hinge.Initialize(back_node, mtruss)
            self.system.Add(constraint_hinge)

            
            box = chrono.ChBodyEasyBox(0.02, 0.04, 0.02, 1000, True, True)  
            
            box.SetPos(end_point + chrono.ChVector3d(0, 0, 0.02))
            box.SetMass(0.1)  
            box.SetInertiaXX(chrono.ChVectorD(1e-4, 1e-4, 1e-4))  
            self.system.Add(box)

            
            
            link_node_box = chrono.ChLinkPointFrame()
            link_node_box.Initialize(back_node, box)
            self.system.Add(link_node_box)

            
            self.end_bodies.append(box)

    def PrintBodyPositions(self):
        print("Positions of end bodies (box bodies) of each chain:")
        for idx, body in enumerate(self.end_bodies):
            pos = body.GetPos()
            print(f"Chain {idx} end body position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")







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
vis.SetWindowTitle('FEA cables - Multiple Chains')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.9, -1))  
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


time_step = 0.01
step_count = 0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(time_step)

    
    step_count += 1
    if step_count % 50 == 0:
        print(f"Step: {step_count}, time: {sys.GetChTime():.2f} s")
        model.PrintBodyPositions()