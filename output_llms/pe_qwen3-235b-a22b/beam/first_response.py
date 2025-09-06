import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize the PyChrono system using SMC for smooth contact handling
sys = chrono.ChSystemSMC()

# 2. Create a mesh for finite element analysis and add it to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# 3. Define beam properties
beam_length = 2.0           # Length of the beam in meters
beam_width_y = 0.1          # Width in Y-direction
beam_width_z = 0.1          # Width in Z-direction
young_modulus = 0.01e9      # Young's modulus in Pascals (flexible material)
shear_modulus = 0.003e9     # Shear modulus (derived from Poisson's ratio)
density = 1000              # Density in kg/m^3

# 4. Configure beam section properties
msection = fea.ChBeamSectionEulerAdvanced()
msection.SetAsRectangularSection(beam_width_y, beam_width_z)
msection.SetYoungModulus(young_modulus)
msection.SetShearModulus(shear_modulus)
msection.SetDensity(density)
msection.SetRayleighDamping(0.000)  # No damping for this simulation

# 5. Create nodes for the beam
# Fixed starting node
node_start = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
node_start.SetFixed(True)
mesh.AddNode(node_start)

# Middle node (for better deformation visualization)
node_mid = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length/2, 0, 0)))
mesh.AddNode(node_mid)

# End node
node_end = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_length, 0, 0)))
mesh.AddNode(node_end)

# 6. Create beam elements connecting the nodes
# First element (start to middle)
element1 = fea.ChElementBeamEuler()
element1.SetNodes(node_start, node_mid)
element1.SetSection(msection)
mesh.AddElement(element1)

# Second element (middle to end)
element2 = fea.ChElementBeamEuler()
element2.SetNodes(node_mid, node_end)
element2.SetSection(msection)
mesh.AddElement(element2)

# 7. Set up visualization using Irrlicht
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Finite Element Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 1))  # Position camera
vis.AddTypicalLights()

# 8. Add visualization for the FEA mesh
mesh_vis = chrono.ChVisualShapeFEA(mesh)
mesh_vis.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM)
mesh_vis.SetColor(chrono.ChColor(0, 0, 1))  # Blue color for beam
mesh.AddVisualShapeFEA(mesh_vis)

# 9. Simulation loop
time_step = 0.001  # 1 millisecond time step
while vis.Run():
    # Advance the simulation by one time step
    sys.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()