import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # 1. Initialize system
    system = chrono.ChSystemSMC()
    system.Set_G_acc(chrono.ChVector3d(0, 0, 0))  # Zero gravity

    # 2. Create IGA beam
    mesh = fea.ChMesh()
    system.Add(mesh)

    # Beam geometry and material properties
    length = 2.0  # Beam length (m)
    num_elements = 8  # Number of finite elements
    density = 7800  # Steel density (kg/m³)
    E = 2.1e11  # Young's modulus (Pa)
    nu = 0.3  # Poisson ratio
    width = 0.1  # Cross-section width (m)
    height = 0.1  # Cross-section height (m)

    # Create section and material
    section = fea.ChBeamSectionEulerAdvanced()
    section.SetDensity(density)
    section.SetYoungModulus(E)
    section.SetShearModulus(E/(2*(1+nu)))
    section.SetAsRectangularSection(width, height)
    section.SetBeamRaleyghDamping(0.01)  # Damping coefficient

    # Build beam along Y-axis
    builder = fea.ChBuilderBeamIGA()
    builder.BuildBeam(mesh, section, num_elements, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, length, 0))
    beam_nodes = builder.GetLastBeamNodes()

    # 3. Create flywheel at center
    flywheel = chrono.ChBodyEasyCylinder(0.15, 0.05, 1000)  # Radius, height, density
    flywheel.SetPos(chrono.ChVector3d(0, length/2, 0))
    flywheel.SetMass(5.0)  # Explicit mass (kg)
    system.Add(flywheel)

    # Connect flywheel to center beam node
    center_node = beam_nodes[int(len(beam_nodes)/2)]
    constraint = chrono.ChLinkNodeFrame()
    constraint.Initialize(center_node, flywheel)
    system.Add(constraint)

    # 4. Create rotational motor at first node
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam_nodes[0], chrono.ChFrameD(chrono.ChVector3d(0, 0, 0)))
    system.Add(motor)

    # Motor speed function (10 rad/s constant rotation)
    motor_speed = chrono.ChFunctionConst(10.0)  # ~95.5 RPM
    motor.SetSpeedFunction(motor_speed)

    # 5. Visualization setup
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(0.5, length/2, 1.5))
    vis.AddTypicalLights()

    # FEM visualization
    beam_visual = fea.ChVisualShapeFEA(mesh)
    beam_visual.SetFEMdataType(fea.FEMDataType.ELEM_BEAM_MZ)
    beam_visual.SetColorscaleMinMax(-40, 40)
    beam_visual.SetSmoothFaces(True)
    beam_visual.SetWireframe(False)
    mesh.AddVisualShapeFEA(beam_visual)

    node_cloud = fea.ChVisualShapeFEA(mesh)
    node_cloud.SetFEMglyphType(fea.FEMGlyphType.NODE_DOT_POS)
    node_cloud.SetFEMdataType(fea.FEMDataType.NONE)
    node_cloud.SetSymbolsThickness(0.01)
    mesh.AddVisualShapeFEA(node_cloud)

    # 6. Simulation loop
    timestep = 0.001
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(timestep)

if __name__ == '__main__':
    main()