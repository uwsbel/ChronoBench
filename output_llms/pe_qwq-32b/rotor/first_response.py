import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math

def main():
    # Initialize system and visualization
    sys = chrono.ChSystemSMC()
    visualSystem = chronoirr.ChVisualSystemIrrlicht()
    visualSystem.SetWindowSize(1280, 1024)
    visualSystem.SetWindowTitle('Jeffcott Rotor Simulation')
    visualSystem.SetSymbolscale(0.01)
    visualSystem.AttachSystem(sys)
    visualSystem.SetCameraPosition(chrono.ChVectorD(1, 1, 3))
    visualSystem.AddTypicalLights()
    visualSystem.Initialize()

    # Create FEA mesh and nodes
    beam_length = 2.0
    num_nodes = 3  # Three nodes at 0, 1, and 2 meters
    mesh = fea.ChMesh()
    nodes = []

    for i in range(num_nodes):
        x = i * (beam_length / (num_nodes - 1))
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x, 0, 0)))
        mesh.AddNode(node)
        nodes.append(node)

    # Create IGA beam element
    element = fea.ChElementBeamIGA()
    element.SetNodes(nodes[0], nodes[1], nodes[2])  # Three nodes for quadratic (degree 2)
    element.SetDegree(2)
    element.SetNumControlPoints(3)

    # Define beam material properties
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetAsRectangularSection(0.05, 0.05)  # Cross-section 5cm x 5cm
    section.SetYoungModulus(2e11)  # Steel Young's modulus
    section.SetShearModulus(8e10)
    section.SetRayleighDamping(0.001)
    element.SetSection(section)

    mesh.AddElement(element)
    mesh.AddToSystem(sys)

    # Create flywheel (rigid body attached to center node)
    flywheel = chrono.ChBody()
    flywheel.SetPos(chrono.ChVectorD(1.0, 0, 0))
    flywheel.SetMass(10.0)  # Flywheel mass
    flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
    flywheel_shape = chrono.ChVisualShapeSphere(0.1)
    flywheel.AddVisualShape(flywheel_shape)
    sys.Add(flywheel)

    # Connect flywheel to center node (nodes[1])
    link_flywheel = chrono.ChLinkLockRevolute()
    link_flywheel.Initialize(flywheel, None, chrono.ChFrameD(nodes[1].GetPos(), chrono.QUNIT))
    link_flywheel.SetAxis(chrono.ChVectorD(1, 0, 0))  # Rotation around X-axis
    sys.Add(link_flywheel)

    # Left end setup (motor-driven)
    left_end_body = chrono.ChBody()
    left_end_body.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.Add(left_end_body)

    link_left = chrono.ChLinkLockRevolute()
    link_left.Initialize(left_end_body, None, chrono.ChFrameD(nodes[0].GetPos(), chrono.QUNIT))
    sys.Add(link_left)

    # Motor between fixed base and left_end_body
    fixed_base = chrono.ChBody()
    fixed_base.SetBodyFixed(True)
    fixed_base.SetPos(chrono.ChVectorD(0, 0, 0))
    sys.Add(fixed_base)

    motor_link = chrono.ChLinkMotorRotationSpeed()
    motor_link.Initialize(fixed_base, left_end_body, chrono.ChFrameD(chrono.ChVectorD(0,0,0), chrono.QUNIT))
    motor_link.SetMotorMode(chrono.ChLinkMotorRotationSpeed.MOTOR_MODE_SPEED)
    motor_link.SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))  # 1 revolution per second (2π rad/s)
    sys.Add(motor_link)

    # Right end setup (fixed bearing)
    right_end_body = chrono.ChBody()
    right_end_body.SetPos(chrono.ChVectorD(2, 0, 0))
    right_end_body.SetBodyFixed(True)
    sys.Add(right_end_body)

    link_right = chrono.ChLinkLockRevolute()
    link_right.Initialize(right_end_body, None, chrono.ChFrameD(nodes[2].GetPos(), chrono.QUNIT))
    sys.Add(link_right)

    # Visualization setup
    mesh.SetAutomaticVisualization(True)
    fixed_base.AddVisualShape(chrono.ChVisualShapeBox(0.1, 0.1, 0.1))
    right_end_body.AddVisualShape(chrono.ChVisualShapeBox(0.1, 0.1, 0.1))
    left_end_body.AddVisualShape(chrono.ChVisualShapeBox(0.1, 0.1, 0.1))

    # Simulation loop
    while visualSystem.Run():
        sys.DoStepDynamics(0.001)
        visualSystem.BeginScene()
        visualSystem.Render()
        visualSystem.EndScene()

if __name__ == '__main__':
    main()