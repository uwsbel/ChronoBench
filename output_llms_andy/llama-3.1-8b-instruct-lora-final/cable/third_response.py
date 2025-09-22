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

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
model.Initialize(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh:
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  # Display moments along the beam
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  # Set color scale for moment visualization
visualizebeamA.SetSmoothFaces(True)  # Enable smooth faces for better visualization
visualizebeamA.SetWireframe(False)  # Set to non-wireframe mode
mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

# Add visualization for node positions:
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
visualizebeamB.SetSymbolsThickness(0.006)  # Set thickness of symbols
visualizebeamB.SetSymbolsScale(0.01)  # Set scale of symbols
visualizebeamB.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
mesh.AddVisualShapeFEA(visualizebeamB)  # Add the node visualization to the mesh

# Create the Irrlicht visualization for rendering
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach Irrlicht to the Chrono system
vis.SetWindowSize(1024, 768)  # Set the size of the rendering window
vis.SetWindowTitle('FEA cables')  # Set the title of the rendering window
vis.Initialize()  # Initialize the visualization
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the window
vis.AddSkyBox()  # Add a skybox for better aesthetics
vis.AddCamera(chrono.ChVector3d(0, 0
print("error happened with only start ```python")