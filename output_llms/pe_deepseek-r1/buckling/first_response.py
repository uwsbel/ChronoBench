import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # =============================================================================
    # System Initialization
    # =============================================================================
    system = chrono.ChSystemSMC()
    system.SetGravity(chrono.ChVector3d(0, -9.81, 0))  # Standard gravity
    
    # =============================================================================
    # FEA Mesh Setup
    # =============================================================================
    mesh = fea.ChMesh()
    system.Add(mesh)
    
    # Material properties (Steel)
    material = fea.ChElasticityEuler()
    material.SetYoungModulus(2.0e11)  # 200 GPa (steel)
    material.SetPoissonRatio(0.3)
    material.SetDensity(7800)         # kg/m³
    
    # Beam cross-section properties
    section = fea.ChBeamSectionEuler()
    section.SetRectangularSection(0.05, 0.01)  # Width x height (5cm x 1cm)
    section.SetMaterial(material)
    
    # =============================================================================
    # Beam Nodes and Elements
    # =============================================================================
    num_elements = 20
    beam_length = 2.0  # meters
    dx = beam_length / num_elements
    
    # Create nodes
    nodes = []
    for i in range(num_elements + 1):
        pos = chrono.ChVector3d(i * dx, 0, 0)
        node = fea.ChNodeFEAxyzrot(chrono.ChFramed())
        node.SetPos(pos)
        nodes.append(node)
        mesh.AddNode(node)
    
    # Create beam elements between nodes
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(section)
        mesh.AddElement(element)
    
    # =============================================================================
    # Boundary Conditions
    # =============================================================================
    # Fix first node completely (cantilever)
    constraint_fixed = fea.ChLinkPointFrame()
    constraint_fixed.Initialize(nodes[0], chrono.ChFrameD(nodes[0].GetPos()))
    system.Add(constraint_fixed)
    
    # Apply compressive load at free end using motor
    motor_function = chrono.ChFunctionRamp(0, -0.1)  # Linear displacement: 0.1 m/s compression
    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(nodes[-1], 
                    chrono.ChFrameD(nodes[-1].GetPos()), 
                    chrono.ChFrameD(nodes[-1].GetPos()))
    motor.SetMotionFunction(motor_function)
    system.Add(motor)
    
    # =============================================================================
    # Visualization Setup
    # =============================================================================
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Beam Buckling Simulation")
    vis.Initialize()
    vis.AddCamera(chrono.ChVector3d(1, 0.5, 2))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    
    # Visualization settings for FEA
    vis_fea = fea.ChVisualizationFEAmesh(mesh)
    vis_fea.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    vis_fea.SetColorscaleMinMax(-10, 10)
    vis_fea.SetSmoothFaces(True)
    mesh.AddVisualizationFEA(vis_fea)
    
    # =============================================================================
    # Solver Configuration
    # =============================================================================
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.GetSolver().AsIterative().SetMaxIterations(100)
    system.GetSolver().AsIterative().SetTolerance(1e-10)
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    
    # =============================================================================
    # Simulation Loop
    # =============================================================================
    time_step = 0.005
    frame_step = 0.02  # Save frame every 20 ms
    frame_number = 0
    
    while vis.Run():
        time = system.GetChTime()
        
        # Start scene
        vis.BeginScene()
        vis.Render()
        
        # Draw additional visual elements
        chronoirr.ChIrrTools.drawGrid(vis, 0.1, 0.1, 20, 20,
                                     chrono.ChCoordsysd(chrono.ChVector3d(1, 0, 0), 
                                     chrono.QUNIT),
                                     chrono.ChColor(0.6, 0.6, 0.8), True)
        
        # Draw motor position
        chronoirr.ChIrrTools.drawSegment(vis, 
                                        nodes[-1].GetPos(), 
                                        nodes[-1].GetPos() + chrono.ChVector3d(0, 0.1, 0),
                                        chrono.ChColor(1, 0, 0), True)
        
        # End scene
        vis.EndScene()
        
        # Advance simulation
        system.DoStepDynamics(time_step)
        
        # Print progress
        if abs(time - frame_number * frame_step) < time_step / 2:
            print(f"Time: {time:.2f}s | Displacement: {nodes[-1].GetPos().x - beam_length:.4f}m")
            frame_number += 1

if __name__ == "__main__":
    main()

# For different buckling shapes:
   section.SetCircularSection(0.02)  # Circular cross-section
   material.SetYoungModulus(1e9)      # Softer material (e.g., plastic)

# Sinusoidal load instead of constant compression:
   motor_function = chrono.ChFunctionSine(0.01, 0.5)  # Amplitude 1cm, frequency 0.5Hz

# Add small initial curvature to trigger specific buckling mode
   nodes[10].SetPos(chrono.ChVector3d(10*dx, 0.001, 0))

# Show axial forces instead of bending moments:
   vis_fea.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_N)