import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    # Initialize Chrono system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Zero gravity

    # Create IGA beam material (elastic)
    beam_material = fea.ChBeamMaterialEuler()
    beam_material.SetDensity(7800)       # Density [kg/m³]
    beam_material.SetYoungModulus(210e9) # Young's modulus [Pa]
    beam_material.SetBeamRaleyghDamping(0.01)  # Damping coefficient

    # Define beam geometry
    length = 2.0       # Beam length [m]
    num_elements = 16  # Number of finite elements
    num_knots = num_elements + 1
    order = 3          # Cubic spline

    # Create IGA beam
    beam = fea.ChBeamIGA()
    beam.SetOrder(order)
    beam.SetMaterial(beam_material)

    # Define control points (straight beam along Y-axis)
    for i in range(num_knots):
        y = i * length / (num_knots - 1)
        point = chrono.ChVectorD(0, y, 0)
        beam.AddPoint(point)
    
    # Build beam geometry
    beam.SetupInitial()

    # Add beam to system
    system.Add(beam)

    # Create flywheel (disk at beam center)
    flywheel = chrono.ChBodyEasyCylinder(0.15, 0.05, 1000)  # Radius, thickness, density
    flywheel.SetPos(chrono.ChVectorD(0, length/2, 0))
    flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))  # Custom inertia
    system.Add(flywheel)

    # Fix flywheel to beam center
    node_mid = beam.GetNode(int(num_knots/2))
    constraint = fea.ChLinkPointFrame()
    constraint.Initialize(node_mid, flywheel)
    system.Add(constraint)

    # Create rotational motor at first node (Y=0)
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam.GetNode(0), chrono.ChFrameD())
    motor_speed = chrono.ChFunction_Ramp(0, 5.0)  # Start at 0 rad/s, ramp 5 rad/s²
    motor.SetSpeedFunction(motor_speed)
    system.Add(motor)

    # Fix last node to ground (Y=length)
    node_end = beam.GetNode(num_knots - 1)
    constraint_fix = fea.ChLinkPointFrame()
    constraint_fix.Initialize(node_end, system.GetGroundBody())
    system.Add(constraint_fix)

    # Set up visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Jeffcott Rotor IGA Simulation')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(1.5, length/2, 1.5), chrono.ChVectorD(0, length/2, 0))
    vis.AddTypicalLights()

    # FEM visualization settings
    beam_vis = fea.ChVisualizationFEAmesh(beam)
    beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
    beam_vis.SetColorscaleMinMax(-100, 100)
    beam_vis.SetSmoothFaces(True)
    beam_vis.SetWireframe(False)
    beam.AddAsset(beam_vis)

    # Runtime settings
    time_step = 0.005
    system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
    system.SetSolverType(chrono.ChSolver.Type_MINRES)
    system.GetSolver().AsIterative().SetMaxIterations(100)
    system.GetSolver().AsIterative().SetTolerance(1e-10)

    # Simulation loop
    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

if __name__ == "__main__":
    main()