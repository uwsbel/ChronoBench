import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr





class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.system = system
        self.mesh = mesh
        self.end_bodies = []

        
        base_section = fea.ChBeamSectionCable()
        base_section.SetDiameter(0.015)         
        base_section.SetYoungModulus(0.01e9)    
        base_section.SetRayleighDamping(0.0001)

        for i in range(n_chains):
            
            builder = fea.ChBuilderCableANCF()

            
            num_elems = 10 + i

            
            y_offset = i * 0.1
            start_pt = chrono.ChVector3d(0, y_offset, 0)
            end_pt   = chrono.ChVector3d(0.5, y_offset, 0)

            
            builder.BuildBeam(
                mesh,
                base_section,
                num_elems,
                start_pt,
                end_pt
            )

            
            nodes = builder.GetLastBeamNodes()
            node_start = nodes[0]
            node_end   = nodes[-1]

            
            mtruss = chrono.ChBody()
            mtruss.SetPos(start_pt)       
            mtruss.SetBodyFixed(True)
            system.Add(mtruss)

            hinge = fea.ChLinkNodeFrame()
            hinge.Initialize(node_start, mtruss)
            system.Add(hinge)

            
            node_start.SetForce(chrono.ChVector3d(0, -5.0, 0))

            
            box = chrono.ChBody()
            box.SetPos(end_pt)
            box.SetMass(0.2)
            
            box.SetInertiaXX(chrono.ChVectorD(0.001, 0.001, 0.001))
            system.Add(box)

            
            sphere_asset = chrono.ChSphereShape(0.025)
            box.AddVisualShape(sphere_asset)

            link_end = fea.ChLinkNodeFrame()
            link_end.Initialize(node_end, box)
            system.Add(link_end)

            
            self.end_bodies.append(box)

        
        system.Add(mesh)

    def PrintBodyPositions(self):
        
        for idx, b in enumerate(self.end_bodies):
            p = b.GetPos()
            print(f"Chain {idx} end‐box pos = ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})")





if __name__ == "__main__":

    
    sys = chrono.ChSystemSMC()
    sys.Set_G_acc(chrono.ChVector3d(0, -9.81, 0))

    
    mesh = fea.ChMesh()

    
    model = Model1(sys, mesh, n_chains=6)

    
    visA = chrono.ChVisualShapeFEA(mesh)
    visA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
    visA.SetColorscaleMinMax(-0.4, 0.4)
    visA.SetSmoothFaces(True)
    mesh.AddVisualShapeFEA(visA)

    visB = chrono.ChVisualShapeFEA(mesh)
    visB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    visB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
    visB.SetSymbolsThickness(0.006)
    visB.SetSymbolsScale(0.01)
    mesh.AddVisualShapeFEA(visB)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Multiple ANCF Cable Chains')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1, 1, 1))
    vis.AddTypicalLights()

    
    solver = chrono.ChSolverMINRES()
    sys.SetSolver(solver)
    if solver.GetType() == chrono.ChSolver.Type_MINRES:
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableDiagonalPreconditioner(True)
        solver.EnableWarmStart(True)
        solver.SetVerbose(False)

    
    ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
    sys.SetTimestepper(ts)

    
    time_step = 0.01
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        sys.DoStepDynamics(time_step)
        
        model.PrintBodyPositions()