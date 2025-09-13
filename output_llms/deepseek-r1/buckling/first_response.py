import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea

def main():
    # 1. Initialize system and set environment
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable gravity

    # 2. Create ground body for reference
    ground = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    ground.SetBodyFixed(True)
    ground.SetPos(chrono.ChVectorD(0, 0, 0))
    system.Add(ground)

    # 3. Create FEA mesh for the beam
    mesh = fea.ChMesh()
    system.Add(mesh)

    # 4. Define beam material and section properties
    density = 7800  # kg/m³ (steel)
    E = 2.0e11      # Pa (Young's modulus)
    nu = 0.3         # Poisson's ratio
    width = 0.02     # m (rectangular cross-section)
    height = 0.01    # m
    
    beam_section = fea.ChBeamSectionEulerAdvanced()
    beam_section.SetDensity(density)
    beam_section.SetYoungModulus(E)
    beam_section.SetShearModulus(E / (2 * (1 + nu)))
    beam_section.SetAsRectangularSection(width, height)
    beam_section.SetRayleighDampingBeta(0.001)  # Damping coefficient

    # 5. Create beam nodes and elements
    length = 1.0    # Beam length (m)
    num_elements = 10
    dx = length / num_elements
    nodes = []

    # Create nodes along the x-axis
    for i in range(num_elements + 1):
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(i * dx, 0, 0)))
        node.SetMass(0)
        mesh.AddNode(node)
        nodes.append(node)

    # Create beam elements between nodes
    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i + 1])
        element.SetSection(beam_section)
        mesh.AddElement(element)

    # 6. Fix the first node (clamped boundary condition)
    constraint_fixed = fea.ChLinkPointFrame()
    constraint_fixed.Initialize(nodes[0], ground)
    system.Add(constraint_fixed)

    # 7. Create slider and constraints at the free end
    slider = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000)
    slider.SetPos(nodes[-1].GetPos())
    system.Add(slider)

    # Prismatic constraint for axial motion only
    prismatic = chrono.ChLinkLockPrismatic()
    prismatic.Initialize(ground, slider, 
                         chrono.ChCoordsysD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)))
    system.Add(prismatic)

    # Connect last node to slider
    constraint_slider = fea.ChLinkPointFrame()
    constraint_slider.Initialize(nodes[-1], slider)
    system.Add(constraint_slider)

    # 8. Define motor function for compressive displacement
    motor_func = chrono.ChFunction_Ramp(0, -0.01)  # Linear displacement: -0.01 m/s
    motor = chrono.ChLinkMotorLinearPosition()
    motor.Initialize(ground, slider, chrono.ChFrameD(nodes[-1].GetPos()))
    motor.SetMotionFunction(motor_func)
    system.Add(motor)

    # 9. Set solver and timestepper
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.SetSolverMaxIterations(100)
    system.SetSolverTolerance(1e-10)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    system.GetSolver().AsIterative().SetVerbose(False)

    # 10. Set up visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("Beam Buckling Simulation")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.5), chrono.ChVectorD(0, 0, 0))
    vis.AddTypicalLights()

    # Visualize beam elements (color maps to axial force)
    beam_vis = fea.ChVisualShapeFEA(mesh)
    beam_vis.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
    beam_vis.SetColorscaleMinMax(-1000, 1000)  # Customize based on expected forces
    beam_vis.SetSmoothFaces(True)
    beam_vis.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam_vis)

    # 11. Simulation loop
    time_step = 0.001
    time_end = 10.0  # Simulate for 10 seconds

    while vis.Run() and system.GetChTime() < time_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()