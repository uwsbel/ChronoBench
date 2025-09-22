import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.beams = []
        self.trusses = []
        self.beam_end_points = []
        self.beam_start_points = []

    def PrintBodyPositions(self, sys):
        for i in range(self.n_chains):
            print(f"Chain {i+1} end body position: {self.beam_end_points[i].GetPos()}")

    def Initialize(self, system, mesh):
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)
        msection_cable2.SetYoungModulus(0.01e9)
        msection_cable2.SetRayleighDamping(0.0001)

        builder = fea.ChBuilderCableANCF()
        for i in range(self.n_chains):
            builder.BuildBeam(
                mesh,
                msection_cable2,
                10 + i,
                chrono.ChVector3d(0, 0, -0.1 + i * 0.1),
                chrono.ChVector3d(0.5, 0, -0.1 + i * 0.1)
            )
            self.beams.append(builder.GetLastBeamNodes())
            self.beam_start_points.append(builder.GetLastBeamNodes().front().GetPos())
            self.beam_end_points.append(builder.GetLastBeamNodes().back().GetPos())

            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)
            self.trusses.append(mtruss)

            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)

            mbody = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
            mbody.SetPos(self.beam_end_points[i])
            system.Add(mbody)
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mbody)
            system.Add(constraint_hinge)

            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
model.Initialize(sys, mesh)
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
vis.AddCamera(chrono.ChVector3d(0, 0
print("error happened with only start ```python")