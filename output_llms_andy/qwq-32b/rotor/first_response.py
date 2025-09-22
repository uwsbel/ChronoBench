import pychrono as chrono
import pychrono.iga as chrono_iga
import pychrono.postprocess as postproc

# Create the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Gravity

# Create the IGA beam
control_points = [
    chrono.ChVectorD(0, 0, 0),
    chrono.ChVectorD(1, 0, 0)
]
knots = [0, 0, 1, 1]  # Linear beam (degree 1)
beam = chrono_iga.ChIgaBeam()
beam.SetDegree(1)
beam.SetControlPoints(control_points)
beam.SetKnots(knots)

# Beam material properties
E = 210e9  # Pa
nu = 0.3
rho = 7800  # kg/m³
beam.SetMaterialYoungModulus(E)
beam.SetMaterialPoissonRatio(nu)
beam.SetDensity(rho)

# Beam cross-section (circular)
cross_section = chrono.ChIgaSectionCircular(radius=0.02)
beam.SetSection(cross_section)

system.Add(beam)

# Create the flywheel
flywheel_radius = 0.1
flywheel_mass = 10.0
flywheel_body = chrono.ChBody()
flywheel_body.SetMass(flywheel_mass)
flywheel_body.SetInertiaXX(chrono.ChVectorD(
    0.5 * flywheel_mass * (flywheel_radius ** 2),
    0.25 * flywheel_mass * (flywheel_radius ** 2),
    0.25 * flywheel_mass * (flywheel_radius ** 2)
))
flywheel_body.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.Add(flywheel_body)

# Attach flywheel to beam's midpoint with a revolute joint
link = chrono.ChLinkLockRevolute()
link.Initialize(flywheel_body, beam, chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0), chrono.QUNIT))
system.Add(link)

# Add rotational motor to drive the beam's end
motor_link = chrono.ChLinkMotorRotationSpeed()
motor_link.Initialize(beam, None, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
motor_link.SetSpeed(10)  # 10 rad/s
system.Add(motor_link)

# Visualization setup
application = chrono.ChIrrApp(system, 'Jeffcott Rotor Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalLights()
application.AddTypicalCamera(chrono.ChVectorD(0.5, 0.5, 0.5), chrono.ChVectorD(0.5, 0, 0))

# Add FEM visualization to the beam
fem_vis = chrono.ChVisualizationFEMMesh()
fem_vis.SetFEMMesh(beam.GetFEMMesh())
fem_vis.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
beam.GetAssets().append(fem_vis)

application.Add(beam)
application.Add(flywheel_body)

application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStepDynamics(0.001)  # Time step
    application.EndScene()