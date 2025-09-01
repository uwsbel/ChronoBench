import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr
import pychrono.mkl as mkl # MKL solver (optional, but good for FEA)
import math
import os

# ---------------------------------------------------------------------
# Custom Motor Function
# This function will define how the compressive displacement is applied over time.
# For example, a slow ramp.
# ---------------------------------------------------------------------
class MyCompressionFunction(chrono.ChFunction):
    def __init__(self, max_displacement, ramp_duration):
        super().__init__()
        self.max_displacement = max_displacement
        self.ramp_duration = ramp_duration
        self.speed = 0
        if self.ramp_duration > 1e-9: # avoid division by zero
            self.speed = self.max_displacement / self.ramp_duration

    def Get_y(self, x): # x is time
        if x <= 0:
            return 0
        elif x < self.ramp_duration:
            return self.speed * x
        else:
            return self.max_displacement

    def Get_y_dx(self, x): # Velocity
        if x <= 0 or x >= self.ramp_duration:
            return 0
        else:
            return self.speed

    def Get_y_dxdx(self, x): # Acceleration
        return 0

# ---------------------------------------------------------------------
# Main Simulation Setup
# ---------------------------------------------------------------------
def main():
    print("Beam Buckling Simulation with PyChrono FEA")

    # Create a Chrono system
    # system = chrono.ChSystemSMC() # Could use SMC, but NSC is fine for this demo
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0)) # Gravity, though buckling is primary here

    # --- Beam Properties ---
    beam_length = 1.0  # meters
    beam_width = 0.02 # meters (for rectangular section)
    beam_thickness = 0.01 # meters (for rectangular section)
    num_elements = 20
    num_nodes = num_elements + 1

    # --- Material Properties (Steel-like) ---
    density = 7850  # kg/m^3
    E_modulus = 210e9  # Pa (Young's Modulus)
    poisson_ratio = 0.3
    # Cross-sectional properties
    area = beam_width * beam_thickness
    Iyy = (beam_width * beam_thickness**3) / 12.0 # Bending about y-axis (local)
    Izz = (beam_thickness * beam_width**3) / 12.0 # Bending about z-axis (local)

    # --- Create FEA Mesh ---
    mesh = fea.ChMesh()
    system.Add(mesh)

    # --- Create FEA Nodes ---
    nodes = []
    for i in range(num_nodes):
        x_pos = (i / num_elements) * beam_length
        node = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(x_pos, 0, 0)))
        mesh.AddNode(node)
        nodes.append(node)

    # --- Add Initial Imperfection (Critical for numerical buckling) ---
    # Without an imperfection, a perfectly symmetric model might not buckle numerically.
    if num_nodes > 2:
        mid_node_idx = num_nodes // 2
        imperfection_mag = beam_thickness * 0.05 # Small percentage of thickness
        nodes[mid_node_idx].SetPos(nodes[mid_node_idx].GetPos() + chrono.ChVectorD(0, imperfection_mag, 0))


    # --- Create FEA Material and Elements ---
    material = fea.ChMaterialBeamEuler()
    material.Set_density(density)
    material.Set_E(E_modulus)
    material.Set_G(E_modulus / (2 * (1 + poisson_ratio))) # Shear modulus (though Euler-Bernoulli neglects shear def.)
    material.Set_Iy(Iyy) # Corresponds to bending in XZ plane (about local y)
    material.Set_Iz(Izz) # Corresponds to bending in XY plane (about local z)
    material.Set_A(area)

    for i in range(num_elements):
        element = fea.ChElementBeamEuler()
        element.SetNodes(nodes[i], nodes[i+1])
        element.SetSectionMaterial(material)
        mesh.AddElement(element)

    # --- Boundary Conditions & Constraints ---
    # 1. Fix the first node (cantilever-like, but other end is pushed)
    nodes[0].SetFixed(True)

    # 2. Apply axial compression to the last node using a motor
    # We'll connect the motor to a simple massless ground body.
    # Alternatively, we could create a small dummy rigid body and link the motor to it.
    ground_body = system.GetGroundBody() # Using the system's ground body

    # Create a marker on the last FEA node (node_frame)
    # The motor will act on this frame.
    # ChLinkMotorLinear needs ChFrame, so we use the node's ChFrame.
    # Forcing a specific direction for compression (e.g. along X)
    # The motor will connect the node's frame to a frame on the ground body.

    # Frame on the ground, initially coincident with the last node's frame
    # This frame will be moved by the motor function.
    motor_ground_frame = chrono.ChFrameD(nodes[-1].GetPos())

    # Define the compression
    max_compression_displacement = -beam_length * 0.15 # Negative for compression along X
    compression_duration = 2.0 # seconds to reach max compression
    compression_func = MyCompressionFunction(max_compression_displacement, compression_duration)

    # Create the linear motor
    # This motor will try to impose a displacement between node[-1] and ground_body
    # along the X-axis of the motor's coordinate system.
    axial_motor = chrono.ChLinkMotorLinearPosition()
    # Initialize motor: master frame (on ground), slave frame (on node)
    # The motor's Z-axis is its actuation axis by default. We want to actuate along X.
    # So, we can set up the motor's reference frame such that its Z is aligned with global X.
    motor_csys = chrono.ChCoordsysD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)) # Motor Z along global X

    # Initialize the motor connecting the ground to the last node's frame
    # The motor applies motion between two ChFrame specified at initialization.
    # Here, we link node[-1]'s ChFrame directly to a ChFrame on the ground.
    # The motor will try to keep the relative position between these two frames
    # equal to the value returned by compression_func.
    axial_motor.Initialize(
        nodes[-1],          # slave body_1 (the FEA node's ChFrame is implicitly used)
        ground_body,        # master body_2 (the ground)
        chrono.ChFrameD(nodes[-1].GetPos(), chrono.Q_from_AngY(chrono.CH_C_PI_2)) # Motor reference frame in abs coords
                                                                        # Its Z axis is the actuation direction
    )
    axial_motor.SetMotionFunction(compression_func)
    system.Add(axial_motor)

    # --- Visualization Setup ---
    vis_mesh = fea.ChVisualShapeFEA(mesh)
    vis_mesh.SetFEMdataType(fea.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ) # Visualize bending moment My
    vis_mesh.SetColorscaleMinMax(-0.1 * E_modulus * Izz / beam_length, 0.1 * E_modulus * Izz / beam_length) # Estimate
    vis_mesh.SetSmoothFaces(True)
    vis_mesh.SetWireframe(False) # Set to True to see wireframe
    vis_mesh.SetBeamResolution(10)
    vis_mesh.SetBeamSectionType(fea.ChVisualShapeFEA.SectionType_RECTANGULAR) # visualize actual section
    vis_mesh.SetScaleBeam(beam_width, beam_thickness) # if rectangular
    mesh.AddVisualShapeFEA(vis_mesh)

    vis_nodes = fea.ChVisualShapeFEA(mesh)
    vis_nodes.SetFEMdataType(fea.ChVisualShapeFEA.DataType_NONE)
    vis_nodes.SetFEMglyphType(fea.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
    vis_nodes.SetSymbolsScale(0.01)
    vis_nodes.SetDefaultSymbolsColor(chrono.ChColor(0.1,0.1,0.8))
    mesh.AddVisualShapeFEA(vis_nodes)


    # --- Irrlicht Visualization Application ---
    irrlicht_app = irr.ChIrrApp(system, "FEA Beam Buckling", irr.dimension2du(1024, 768))
    irrlicht_app.AddTypicalLights()
    irrlicht_app.AddTypicalSky()
    irrlicht_app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    irrlicht_app.AddTypicalCamera(irr.vector3df(beam_length * 0.5, beam_length * 0.3, -beam_length * 0.7), # camera position
                                  irr.vector3df(beam_length * 0.5, 0, 0))      # "look at" point
    irrlicht_app.AssetBindAll()
    irrlicht_app.AssetUpdateAll()


    # --- Solver and Timestepper ---
    # Use MKL solver if available (recommended for FEA)
    try:
        mkl_solver = mkl.ChSolverPardisoMKL()
        mkl_solver.LockSparsityPattern(True) # Important for FEA performance
        system.SetSolver(mkl_solver)
        print("Using MKL Pardiso Solver.")
    except Exception as e:
        print(f"MKL solver not available or failed to initialize: {e}")
        print("Using MINRES solver as fallback.")
        solver = chrono.ChSolverMINRES()
        system.SetSolver(solver)
        solver.SetMaxIterations(200)
        solver.SetTolerance(1e-10)
        solver.EnableWarmStart(True)
        solver.SetVerbose(False)


    # Timestepper (HHT - Hilber-Hughes-Taylor - good for FEA)
    system.SetTimestepperType(chrono.ChTimestepper.Type_HHT)
    timestepper = system.GetTimestepper().CastToChTimestepperHHT()
    timestepper.SetAlpha(-0.2)  # Numerical damping (typical range -0.33 to 0)
    timestepper.SetMaxiters(10)
    timestepper.SetAbsTolerances(1e-5, 1e-4) # abs_pos_tol, abs_vel_tol
    timestepper.SetMode(chrono.ChTimestepperHHT.ACCELERATION)
    timestepper.SetStepControl(False) # Use fixed step size for this demo
    timestepper.SetVerbose(False)

    # --- Simulation Loop ---
    timestep = 0.002 # simulation time step
    simulation_end_time = compression_duration + 1.0 # simulate a bit longer after compression stops

    irrlicht_app.SetTimestep(timestep)
    irrlicht_app.SetTryRealtime(False) # Run as fast as possible

    max_y_displacement = 0

    while irrlicht_app.GetDevice().run():
        current_time = system.GetChTime()
        if current_time > simulation_end_time:
            break

        irrlicht_app.BeginScene(True, True, chrono.ChColor(0.2, 0.25, 0.3))
        irrlicht_app.DrawAll()

        # Print some info (optional)
        if system.GetNumcontacts() > 0 : # Should be 0 for this specific setup
            print(f"Time: {current_time:.4f} s, Contacts: {system.GetNumcontacts()}")

        # Check mid-node displacement for buckling indication
        mid_node_y_disp = nodes[mid_node_idx].GetPos().y
        if abs(mid_node_y_disp) > max_y_displacement:
            max_y_displacement = abs(mid_node_y_disp)

        # Add custom text to Irrlicht window
        font = irrlicht_app.GetIGUIEnvironment().getBuiltInFont()
        text_pos = irr.position2di(10,10)
        text_color = irr.SColor(255, 255,255,0) # ARGB
        font.draw(f"Time: {current_time:.3f} s", text_pos, text_color)
        text_pos.Y += 20
        font.draw(f"Max Y-Disp (mid): {max_y_displacement:.4f} m", text_pos, text_color)
        text_pos.Y += 20
        applied_disp = compression_func.Get_y(current_time)
        font.draw(f"Applied X-Disp: {applied_disp:.4f} m", text_pos, text_color)

        # Perform the dynamic step
        irrlicht_app.DoStep() # This internally calls system.DoStepDynamics(timestep)

        irrlicht_app.EndScene()

    print(f"Simulation finished. Max mid-node Y displacement: {max_y_displacement:.4f} m")
    irrlicht_app.GetDevice().closeDevice()


if __name__ == "__main__":
    # Ensure Chrono data directory is set for assets (like logo)
    chrono.SetChronoDataPath(os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/'))
    main()