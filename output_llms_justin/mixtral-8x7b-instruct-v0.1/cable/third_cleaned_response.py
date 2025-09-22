import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh):
        self.n_chains = 6  
        self.beam_elements = []
        self.truss_bodies = []
        self.box_bodies = []
        self.hinge_constraints = []
        self.box_constraints = []

        self.beam_spacing = 0.15  
        self.beam_length = 1.0  
        self.beam_elements_per_chain = 10  
        self.beam_diameter = 0.015  
        self.beam_young_modulus = 0.01e9  
        self.beam_rayleigh_damping = 0.0001  

        self.box_size = 0.1  
        self.box_young_modulus = 1e9  
        self.box_density = 1000  

        self.gravity = chrono.ChVectorD(0, -0.7, 0)  

        self.step = 0

        for i in range(self.n_chains):
            self.create_chain(i, system, mesh)

    def create_chain(self, chain_index, system, mesh):
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(self.beam_diameter)  
        msection_cable2.SetYoungModulus(self.beam_young_modulus)  
        msection_cable2.SetRayleighDamping(self.beam_rayleigh_damping)  

        
        builder = fea.ChBuilderCableANCF()

        
        start_point = chrono.ChVectorD(0, 0, -0.1 + chain_index * self.beam_spacing)
        end_point = start_point.Copy()
        end_point.x += self.beam_length

        
        builder.BuildBeam(
            mesh,  
            msection_cable2,  
            self.beam_elements_per_chain,  
            start_point,  
            end_point  
        )

        
        last_nodes = builder.GetLastBeamNodes()

        
        last_nodes.front().SetForce(self.gravity)

        
        mtruss = chrono.ChBody()
        mtruss.SetFixed(True)  

        
        constraint_hinge = fea.ChLinkNodeFrame()
        constraint_hinge.Initialize(last_nodes.back(), mtruss)
        system.Add(constraint_hinge)  
        self.hinge_constraints.append(constraint_hinge)

        
        mbox = chrono.ChBody()
        mbox.SetBox(chrono.ChVectorD(self.box_size / 2), self.box_size, self.box_size, self.box_size, True)
        mbox.SetMass(self.box_size ** 3 * self.box_density)
        mbox.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
        mbox.SetPos(end_point)

        
        visualizebox = chrono.ChVisualBox(self.box_size, self.box_size, self.box_size, chrono.ChColor(1, 0, 0))
        mbox.AddVisualShape(visualizebox)

        
        system.Add(mbox)
        self.box_bodies.append(mbox)

        
        constraint_box = fea.ChLinkNodeFrame()
        constraint_box.Initialize(last_nodes.back(), mbox)
        system.Add(constraint_box)  
        self.box_constraints.append(constraint_box)

    def PrintBodyPositions(self, system):
        for i in range(self.n_chains):
            body = self.box_bodies[i]
            pos = body.GetPos()
            print(f"Chain {i + 1} end body position: {pos}")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
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


visualizebox = chrono.ChVisualBox(model.box_size, model.box_size, model.box_size, chrono.ChColor(1, 0, 0))
for box in model.box_bodies:
    box.AddVisualShape(visualizebox)


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
    if model.step % 10 == 0:
        model.PrintBodyPositions(sys)
    model.step += 1