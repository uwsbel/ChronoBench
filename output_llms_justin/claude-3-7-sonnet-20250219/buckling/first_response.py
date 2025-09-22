# Import required modules
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np
import math

# Initialize the simulation system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Set solver parameters for accuracy
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(150)
system.SetMaxPenetrationRecoverySpeed(0.1)
system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
timestepper = chrono.ChTimestepperHHT(system)
timestepper.SetAlpha(-0.2)  # HHT parameter
timestepper.SetMaxiters(5)
timestepper.SetAbsTolerances(1e-5, 1e-3)

# Create a mesh for the beam FEA
mesh = fea.ChMesh()

# Set material properties for the beam
beam_young_modulus = 2.1e11  # Steel Young's modulus
beam_poisson_ratio = 0.3     # Steel Poisson ratio
beam_density = 7800          # Steel density
beam_section = 0.01 * 0.01   # Cross-sectional area (square beam)
beam_length = 1.0            # Beam length
beam_Izz = (0.01**4) / 12    # Moment of inertia

# Create the beam material
beam_material = fea.ChElasticityCosseratSimple()
beam_material.SetYoungModulus(beam_young_modulus)
beam_material.SetGshearModulus(beam_young_modulus / (2 * (1 + beam_poisson_ratio)))
beam_material.SetBeamRaleyghDamping(0.01)

# Create the beam section
beam_section_props = fea.ChBeamSectionCosserat(beam_material)
beam_section_props.SetAsRectangularSection(0.01, 0.01)
beam_section_props.SetDensity(beam_density)

# Create the beam nodes
beam_elements = 20  # Number of beam elements
nodes = []

for i in range(beam_elements + 1):
    x = i * (beam_length / beam_elements)
    node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
    mesh.AddNode(node)
    nodes.append(node)
    
    # Fix the first node (clamped beam)
    if i == 0:
        constraint = fea.ChLinkNodeFrame()
        constraint.Initialize(node, system.GetBodyList()[0])
        system.Add(constraint)

# Create beam elements and connect to nodes
for i in range(beam_elements):
    element = fea.ChElementBeamEuler()
    element.SetNodes(nodes[i], nodes[i+1])
    element.SetSection(beam_section_props)
    mesh.AddElement(element)

# Add the mesh to the system
system.Add(mesh)

# Create a ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
system.Add(ground)

# Create a movable plate at the end of the beam
end_plate = chrono.ChBody()
end_plate.SetPos(chrono.ChVectorD(beam_length, 0, 0))
end_plate.SetMass(1.0)
end_plate.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
end_plate_vis = chrono.ChBoxShape()
end_plate_vis.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.05, 0.05, 0.05))
end_plate.AddVisualShape(end_plate_vis)
system.Add(end_plate)

# Connect the last node to the end plate
end_constraint = fea.ChLinkNodeFrame()
end_constraint.Initialize(nodes[-1], end_plate)
system.Add(end_constraint)

# Create a custom motor function to apply compressive force
class MyCompressionMotor(chrono.ChFunction):
    def __init__(self, speed):
        super(MyCompressionMotor, self).__init__()
        self.speed = speed
        
    def Get_y(self, x):
        # Gradually increase the compression over time
        if x < 2.0:
            return 0
        else:
            return -self.speed * (x - 2.0)

# Add a linear actuator to compress the beam
actuator = chrono.ChLinkLinActuator()
actuator.Initialize(ground, end_plate, chrono.ChCoordsysD(chrono.ChVectorD(beam_length, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
compression_function = MyCompressionMotor(0.03)  # Compression speed in m/s
actuator.SetDistanceFunction(compression_function)
system.Add(actuator)

# Add a small lateral force to help initiate buckling
lateral_force = chrono.ChForce()
lateral_force.SetF_x(0)
lateral_force.SetF_y(0.5)  # Small lateral force
lateral_force.SetF_z(0)
nodes[int(beam_elements/2)].AddForce(lateral_force)

# Create the visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Beam Buckling Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0.5, 0.2, 1.0), chrono.ChVectorD(0.5, 0, 0))
vis.AddTypicalLights()

# Enable visualization of forces
vis.AddVisualModel(mesh, True)

# Add FEA visualization
beam_vis = fea.ChVisualizationFEAmesh(mesh)
beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
beam_vis.SetColorscaleMinMax(-0.01, 0.01)
beam_vis.SetSmoothFaces(True)
beam_vis.SetWireframe(False)
mesh.AddVisualModel(beam_vis)

beam_vis_2 = fea.ChVisualizationFEAmesh(mesh)
beam_vis_2.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
beam_vis_2.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
beam_vis_2.SetSymbolsThickness(0.006)
beam_vis_2.SetSymbolsScale(0.01)
beam_vis_2.SetZbufferHide(False)
mesh.AddVisualModel(beam_vis_2)

# Simulation settings
time_step = 0.001
simulation_duration = 10.0  # Seconds

# Run the simulation
time = 0
while time < simulation_duration and vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    system.DoStepDynamics(time_step)
    time += time_step
    
    # Log data (optional)
    if int(time / time_step) % 100 == 0:
        print(f"Time: {time:.3f} s, End displacement: {end_plate.GetPos().x - beam_length:.6f} m")

# Clean up
vis.Dispose()