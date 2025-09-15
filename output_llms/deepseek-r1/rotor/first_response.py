import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 1. Initialize system
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Zero gravity

# 2. Create IGA beam
length = 2.0  # Beam length (m)
n_elements = 32  # Number of elements
order = 2  # NURBS order

# Beam section properties
density = 7800  # kg/m³
E = 200e9  # Young's modulus (Pa)
nu = 0.3  # Poisson's ratio
diameter = 0.05  # m
area = chrono.CH_C_PI * (diameter/2)**2
Iyy = chrono.CH_C_PI * (diameter**4)/64
section = fea.ChBeamSectionEuler()
section.SetAsCircularSection(diameter)
section.SetDensity(density)
section.SetYoungModulus(E)

# Create geometry and mesh
path = fea.ChLineBezier(chrono.ChMatrixDynamicD(2, 3))
path.SetPoint(0, chrono.ChVectorD(0, 0, 0))
path.SetPoint(1, chrono.ChVectorD(0, 0, length))

mesh = fea.ChMesh()
beam = fea.ChBeamIGA()
beam.SetupGeometry(path, n_elements, order)
beam.SetSection(section)
beam.SetSectionRotation(0)  # No rotation
mesh.AddElement(beam)
system.Add(mesh)

# 3. Add flywheel at center
flywheel = chrono.ChBodyEasyCylinder(0.15, 0.05, 7800)  # Radius, height, density
flywheel.SetPos(chrono.ChVectorD(0, 0, length/2))
system.Add(flywheel)

# Connect flywheel to beam center node
node_mid = beam.GetNode(int(n_elements/2))
constraint = fea.ChLinkPointFrame()
constraint.Initialize(node_mid, flywheel)
system.Add(constraint)

# 4. Apply rotation at fixed end
motor = chrono.ChLinkMotorRotationSpeed()
motor_shaft = chrono.ChBody()
motor_shaft.SetBodyFixed(True)
system.Add(motor_shaft)

motor.Initialize(motor_shaft, beam.GetNode(0).GetBody(),
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Ramp(0, 2.0))  # 2 rad/s² acceleration
system.Add(motor)

# 5. Visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Jeffcott Rotor - IGA Beam')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.5, 0.5, 1.0))
vis.AddTypicalLights()

# FEM visualization settings
vis_fem = fea.ChVisualizationFEAmesh(mesh)
vis_fem.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
vis_fem.SetColorscaleMinMax(0, 5)
mesh.AddVisualizationFEA(vis_fem)

# Simulation loop
time_step = 0.001
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)