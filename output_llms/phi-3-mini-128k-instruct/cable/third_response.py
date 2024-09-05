import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# ----------------------------------------------------------------------------
# Model1: A series of chains composed of ANCF cable elements, each with a
# hinged end to the ground, moving under gravity alone.
# This model demonstrates the use of the utility class ChBuilderCableANCF and
# the creation of multiple chains with connected bodies.
# ----------------------------------------------------------------------------

class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chains = []
        self.end_positions = []
        self.system = system
        self.mesh = mesh
        self.create_chains()

    def create_chains(self):
        for i in range(self.n_chains):
            # Create a truss body (a fixed reference frame in the simulation)
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  # Fix the truss body

            # Create and initialize a hinge constraint to fix beam's end point to the truss
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(self.mesh.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)  # Add the constraint to the system

            # Create a chain of ANCF cable elements
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  # Set the diameter of the cable section to 15 mm
            msection_cable2.SetYoungModulus(0.01e9)  # Set the Young's modulus of the cable section (0.01 GPa)
            msection_cable2.SetRayleighDamping(0.0001)  # Set Rayleigh damping to zero for this section
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,  # The mesh to which the created nodes and elements will be added
                msection_cable2,  # The beam section properties to use
                i + 1,  # Number of ANCF elements to create along the beam (increases with each chain)
                chrono.ChVector3d(0, 0, -0.1 - 0.01 * i),  # Starting point ('A' point) of the beam
                chrono.ChVector3d(0.5 + 0.01 * i, 0, -0.1 - 0.01 * i)  # Ending point ('B' point) of the beam
            )

            # Connect the beam's endpoint to a chrono body (box) and establish further constraints
            mbox = chrono.ChBox()
            mbox.SetSize(chrono.ChVector3d(0.1, 0.1, 0.1))
            mbox.SetPos(self.mesh.GetLastBeamNodes().back().GetPos())
            self.system.Add(mbox)
            self.system.Add(constraint_hinge)

            # Store the end position for visualization
            self.end_positions.append(self.mesh.GetLastBeamNodes().back().GetPos())

            # Add visualization for the FEM mesh
            visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
            visualizebeamA.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  # Display nodes as dots
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  # No additional FEM data visualization
            visualizebeamA.SetSymbolsThickness(0.006)  # Set thickness of symbols
            visualizebeamA.SetSymbolsScale(0.01)  # Set scale of symbols
            visualizebeamA.SetZbufferHide(False)  # Ensure symbols are not hidden by z-buffer
            self.mesh.AddVisualShapeFEA(visualizebeamA)  # Add the visualization shape to the mesh

            # Add visualization for the chrono body (box)
            visualizebox = chrono.ChVisualShapeFEA(self.mesh)
            visualizebox.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BOX)
            visualizebox.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualizebox.SetSymbolsThickness(0.006)
            visualizebox.SetSymbolsScale(0.01)
            visualizebox.SetZbufferHide(False)
            self.mesh.AddVisualShapeFEA(visualizebox)

    def PrintBodyPositions(self):
        for i, pos in enumerate(self.end_positions):
            print(f"Chain {i+1} end body position: {pos}")

# Initialize the physical system and mesh container:
sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()

# Create the model and add the mesh to the system
model = Model1(sys, mesh)
sys.Add(mesh)  # Remember to add the mesh to the physical system

# Add visualization for the FEM mesh:
# This allows visualization of the forces/moments in the beam elements:
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
vis.AddLogo(chrono.GetChrono















































,,














































,











,














































,






























































































,

































,
















































,
























































,


,






,
,

,
,







,
,
,
,
,
,


,
,

,
,
,
,
,
,
,
,
,
,


,
,

,
,
,
,

,

,
,
,
,
,
,
,
,
,
,
,
,
,
,
,,,
,
,
,
,
,



,






,
,

,


,
,
,










,








,









,




,

,


,






,


















,










































































































































,
,
,









































,































,
,





















































































,



,

















































,
















,



































































,





,
,






,




,
,











,
,











,


,
,
,
,
,
,
,
,













 of












,




















































































,


















































,






,






,








,





,
,












,

















,





,






















































































,











































































































,






























































.



















,
,










,












































,





,



















,



,















































,
,
























































,























,
,,




,

























,
,
,
,


















,
























,

































































































































































































































































































,
,




,

















,















































































































,





,
,




,






























,
,








,
,
,

,





,
,)

,
,











,










,
,
,



,

































,




,




,
,

,
































,



,
,
,






















,
,

,
,
,






































,

,



















































































,























































































































,





,





















,



























































,
,







,










,








,
,







,




,

,
,
,


,
,
,
,
,
,




,
,

,
,

,
,
,



,
,



,
,

,
,
,
,
,
,
,
,
,

,
,,
,


,
,



,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,















,




,


,

,


,




,

,













,
,
,
,



















,




,

,









,
,

















,













,
,




,

















   





,















,
,




































,
,
,
,
,



,





,











,












,
,
,
,


,

,
,
,


,










,

,
,
,

,
,
,

,









,






,
,


,
,
,
,
,
,







,
,
,
,
,
,





,


,

,
,
,
,
,
,
,
,


,
,





















,

,
,












,
,
,
,
,
















,
,
,
,
,
,



,
,
,
,
,











,




,
,

















,


,
,













,
,





,

,
,
,
,,
,

















,
























,
,























,





,



































































































































































.
.




,
,


,
,
,
























,
,






























,





































































































,


,
,
,
,
,


.
























































































































,
,









,
,





























,
,





,








,































,
,




,
,




























,
,






























































































































,
,


























,









,
,










,



















,
,
,
,

































,







































,
,
,
,





,

















,
,
,
,
,
,


,
,
























,
,
,
,
,
,
,
,
,
,
,
,


























,
,
,
,
,
,


,
,

















,


,
,
,
,
,,,,
,,,,,,
,
,,






















,
,
,
,,
,
,




,





















,
,
,
,
,
,
,
,
,,,,,,,,














,





,

,
,
,
,


























,
,
,





,















,































,


























,
,
,
,
,
,
,







)
























,
,
,
,,,,
,
,
,





















,
,
,
,







,
,









































































,
,

,
,






































,

)

,
,














,
,
,

















,
,


,











,
,
,















,








,
,
,
,


,





,


,
,



,

,
,
,
























,
,
,
,








































,
,
,
,
,






























,





















































































,
,





























,
,






























,


















,























































































,
print("error happened with only start ```python")