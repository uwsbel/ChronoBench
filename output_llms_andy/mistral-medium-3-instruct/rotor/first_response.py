import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.iga as iga

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set the path to Chrono data directory

# Create a physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Create the IGA beam (Jeffcott rotor)
beam_length = 2.0
beam_radius = 0.05
beam_density = 7800  # kg/m^3 (steel)
beam_young_modulus = 210e9  # Pa (steel)
beam_poisson_ratio = 0.3

# Create IGA beam
beam = iga.ChBeamIGA()
beam.SetLength(beam_length)
beam.SetRadius(beam_radius)
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_young_modulus)
beam.SetPoissonRatio(beam_poisson_ratio)

# Set IGA parameters (degree and number of spans)
beam.SetDegree(2)  # Quadratic basis functions
beam.SetNumSpans(10)  # Number of elements

# Add beam to system
system.Add(beam)

# 3. Create the flywheel (central mass)
flywheel_mass = 10.0
flywheel_radius = 0.2
flywheel_inertia = chrono.ChVectorD(flywheel_mass*flywheel_radius*flywheel_radius/2,
                                   flywheel_mass*flywheel_radius*flywheel_radius/4,
                                   flywheel_mass*flywheel_radius*flywheel_radius/4)

flywheel = chrono.ChBody()
flywheel.SetMass(flywheel_mass)
flywheel.SetInertiaXX(flywheel_inertia)
flywheel.SetPos(chrono.ChVectorD(beam_length/2, 0, 0))

# Create flywheel visualization
flywheel_vis = chrono.ChSphereShape()
flywheel_vis.GetSphereGeometry().rad = flywheel_radius
flywheel.AddVisualShape(flywheel_vis)

system.Add(flywheel)

# 4. Create the rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(beam.GetNode(0), beam.GetNode(0), chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(100))  # 100 rad/s constant speed
system.AddLink(motor)

# 5. Connect the flywheel to the beam center
link_flywheel = chrono.ChLinkLockLock()
link_flywheel.Initialize(beam.GetNode(beam.GetNumNodes()//2), flywheel)
system.AddLink(link_flywheel)

# 6. Boundary conditions (fix one end)
beam.GetNode(0).SetFixed(True)

# 7. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 5))
vis.AddTypicalLights()

# Add FEM visualization for the beam
beam_vis = fea.ChVisualizationFEAmesh(beam.GetMesh())
beam_vis.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_STRESS_VONMISES)
beam_vis.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
beam_vis.SetFEMrenderingMode(fea.ChVisualizationFEAmesh.E_RENDER_CONTOUR3)
beam_vis.SetSymbolsThickness(0.01)
beam_vis.SetSymbolsScale(1)
beam_vis.SetDefaultColors()
vis.AddVisualModel(beam_vis)

# 8. Simulation loop
time_step = 0.001
end_time = 5.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)

    if system.GetChTime() > end_time:
        break