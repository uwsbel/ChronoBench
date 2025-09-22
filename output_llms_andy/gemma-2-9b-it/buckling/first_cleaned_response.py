import pychrono as chrono
import pychrono.irrlicht as vis


chrono.SetChronoDataPath("data/")
system = chrono.ChSystemNSC()


beam_material = chrono.ChMaterialSurfaceNSC()
beam_material.SetFriction(0.5, 0.5)
beam_material.SetRestitution(0.0)
beam_element_type = chrono.ChLinkSegmentFEA


beam_segment1 = chrono.ChBodyEasyBox(1.0, 0.1, 0.01, beam_material)
beam_segment1.SetPos(chrono.ChVectorD(0, 0, 0))

beam_segment2 = chrono.ChBodyEasyBox(1.0, 0.1, 0.01, beam_material)
beam_segment2.SetPos(chrono.ChVectorD(1.0, 0, 0))


def motor_torque(time, position):
    return 10 * (position - 0.5)


motor = chrono.ChMotorRotation(
    chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT()),
    beam_segment1.GetFrame_m(),
    chrono.ChVectorD(0, 0, 1),
    motor_torque
)
beam_segment1.AddMotor(motor)


constraint = chrono.ChLinkLock(beam_segment1, beam_segment2, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0, 0), chrono.QUNIT()))
system.Add(constraint)


vis_app = vis.ChVisualSystemIrrlicht(
    width=800,
    height=600,
    vs_window_title="Beam Buckling Simulation",
    far_plane=10000,
)
vis_app.Add(beam_segment1)
vis_app.Add(beam_segment2)


system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.Set_Damping_factor(0.1)
system.SetSolverType(chrono.ChSolverMode.CH_SOLVER_MIN_POSITION_ERROR)
system.SetTimestepperType(chrono.ChTimestepperMode.CH_TIMESTEPPER_RK4_WITH_DAMPING)


while vis_app.Run():
    system.DoStepDynamics(1e-3)
    vis_app.Render()