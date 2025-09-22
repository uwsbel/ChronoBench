import pychrono as chrono
import pychrono.irrlicht as chr

def main():
    # Initialize system and visualization
    system = chrono.ChSystem()
    vis = chr.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Beam Buckling FEA Simulation')
    vis.SetCameraPosition(chrono.ChVectorD(3, 3, 3))
    vis.AddLight(chrono.ChVectorD(3, 3, 3), chrono.ChVectorD(255, 255, 255))
    system.SetVisualSystem(vis)
    vis.SetSystem(system)

    # Beam parameters
    length = 2.0
    num_elements = 20
    width = 0.1  # Cross-section width
    height = 0.05  # Cross-section height
    young_modulus = 210e9  # Steel
    poisson = 0.3
    density = 7850  # Steel density

    # Create beam FEA object
    beam = chrono.fea.ChBeamFEAobject()
    beam.SetLength(length)
    beam.SetElementsNum(num_elements)

    # Define structural material properties
    beam_material = chrono.fea.ChBeamMaterial()
    beam_material.SetYoungModulus(young_modulus)
    beam_material.SetPoissongRatio(poisson)
    beam_material.SetDensity(density)

    # Set beam section with material
    section = chrono.fea.ChBeamSectionRBE3(width, height)
    section.SetMaterial(beam_material)
    beam.SetSection(section)

    system.Add(beam)

    # Ground body for constraints
    ground = chrono.ChBody()
    ground.SetBodyFixed(True)
    system.Add(ground)

    # Fix first node to ground
    first_node = beam.GetNode(0)
    constraint = chrono.ChLinkLockPointFrame()
    constraint.Initialize(first_node, ground, chrono.ChFrameD(first_node.GetPos()))
    system.Add(constraint)

    # Dummy body at original end position
    dummy = chrono.ChBody()
    dummy.SetBodyFixed(True)
    dummy.SetPos(chrono.ChVectorD(length, 0, 0))
    system.Add(dummy)

    # Motor to apply compressive displacement
    last_node = beam.GetNode(num_elements - 1)
    motor = chrono.ChLinkMotorLinear()
    motor.Initialize(last_node, dummy, chrono.ChFrameD())
    # Ramp function: displacement decreases over time (compressive)
    motor.SetMotionFunction(chrono.ChFunction_Ramp(0, -0.01))  # slope = -0.01 m/s
    system.Add(motor)

    # Solver and timestepper setup
    timestepper = chrono.ChTimestepperHHT(system)
    timestepper.SetAlpha(-0.2)  # HHT damping parameter
    timestepper.SetMaxiters(5)
    timestepper.SetAbsTolerances(1e-5)
    timestepper.SetRelTolerances(1e-4)
    system.SetTimestepper(timestepper)

    # Simulation loop
    time = 0
    while time < 10:
        system.DoStepDynamics(1e-3)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        time += 1e-3

if __name__ == '__main__':
    main()