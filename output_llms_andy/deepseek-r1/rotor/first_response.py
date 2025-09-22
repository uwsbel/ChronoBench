import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irrlicht
import pychrono.vehicle as vehicle

def main():
    # Initialize system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Zero gravity

    # Beam parameters
    length = 2.0  # Beam length (m)
    num_control_points = 5  # Control points for IGA beam
    order = 2  # Quadratic basis functions

    # Create IGA beam section (circular cross-section)
    beam_section = fea.ChBeamSectionEuler()
    beam_diameter = 0.05  # 5 cm diameter
    beam_area = 3.1416 * (beam_diameter ** 2) / 4.0
    beam_material = chrono.ChMaterialSurfaceNSC()
    beam_section.SetDensity(7800)  # Steel density (kg/m³)
    beam_section.SetYoungModulus(210e9)  # Steel Young's modulus (Pa)
    beam_section.SetGshearModulus(80e9)  # Steel shear modulus
    beam_section.SetBeamRaleyghDamping(0.01)  # Damping coefficient
    beam_section.SetAsCircularSection(beam_diameter)

    # Build IGA beam
    builder = fea.ChBuilderBeamIGA()
    builder.BuildBeam(system,                     # System
                      beam_section,               # Section properties
                      num_control_points,         # Number of control points
                      chrono.ChVectorD(0, 0, 0),  # Start point
                      chrono.ChVectorD(0, 0, length),  # End point
                      order)                      # Curve order
    beam = builder.GetLastBeam()

    # Create flywheel at beam center
    flywheel_radius = 0.3
    flywheel_thickness = 0.1
    flywheel = chrono.ChBodyEasyCylinder(flywheel_radius, flywheel_thickness,  # Dimensions
                                         7800,  # Density (kg/m³)
                                         True,  # Visualization
                                         True)  # Collision
    flywheel.SetPos(chrono.ChVectorD(0, 0, length / 2))  # Center position
    system.Add(flywheel)

    # Attach flywheel to center beam node
    center_node_idx = num_control_points // 2
    center_node = beam.GetNode(center_node_idx)
    constraint = chrono.ChLinkPointFrame()
    constraint.Initialize(center_node, flywheel)
    system.Add(constraint)

    # Create rotational motor at first node (bottom end)
    motor_function = chrono.ChFunction_Ramp(0, 10.0)  # Start at 0, angular speed 10 rad/s
    motor = chrono.ChLinkMotorRotationAngle()
    motor.Initialize(beam.GetNode(0),  # Node to rotate
                     flywheel,         # Reference frame (fixed to ground)
                     chrono.ChFrameD(chrono.ChVectorD(0, 0, 0))  # Motor position
    )
    motor.SetAngleFunction(motor_function)
    system.Add(motor)

    # Visualization setup
    vis = irrlicht.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Jeffcott Rotor Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.0), chrono.ChVectorD(0, 0, length / 2))
    vis.AddTypicalLights()

    # FEM visualization settings
    vis_fem = fea.ChVisualizationFEAmesh(beam)
    vis_fem.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    vis_fem.SetColorscaleMinMax(-500, 500)
    vis_fem.SetSmoothFaces(True)
    vis_fem.SetWireframe(False)
    beam.AddAsset(vis_fem)

    # Runtime parameters
    time_step = 0.001
    time_end = 10.0
    time = 0

    # Simulation loop
    while vis.Run() and time < time_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)
        time += time_step

if __name__ == "__main__":
    main()