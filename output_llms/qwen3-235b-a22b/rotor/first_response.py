import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as irr

# Initialize the Chrono system
sys = chrono.ChSystemNSC()

# Create the IGA beam mesh
mesh = fea.ChMesh()

# Material properties for the beam
material = fea.ChIgaMaterialLinearElastic()
material.E = 210e9  # Young's modulus (Pa)
material.nu = 0.3    # Poisson's ratio
material.rho = 7800  # Density (kg/m^3)

# Beam cross-section parameters (circular cross-section)
radius = 0.01  # 1 cm radius
area = chrono.CH_C_PI * radius**2
Iyy = 0.25 * chrono.CH_C_PI * radius**4  # Area moment of inertia (y-axis)
Izz = Iyy  # Symmetric cross-section
J = 0.5 * chrono.CH_C_PI * radius**4     # Polar moment of inertia

# Create beam nodes (0, 0.5, and 1 meter along x-axis)
nodeA = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
nodeC = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0.5, 0, 0)))
nodeB = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(1, 0, 0)))

mesh.AddNode(nodeA)
mesh.AddNode(nodeC)
mesh.AddNode(nodeB)

# Create IGA beam elements (two elements: A-C and C-B)
element1 = fea.ChIgaElement1D()
element1.SetNodes(nodeA, nodeC)
element1.SetMaterial(material)
element1.SetArea(area)
element1.SetIyy(Iyy)
element1.SetIzz(Izz)
element1.SetJ(J)
mesh.AddElement(element1)

element2 = fea.ChIgaElement1D()
element2.SetNodes(nodeC, nodeB)
element2.SetMaterial(material)
element2.SetArea(area)
element2.SetIyy(Iyy)
element2.SetIzz(Izz)
element2.SetJ(J)
mesh.AddElement(element2)

# Add mesh to the system
sys.Add(mesh)

# Create the flywheel (mass and inertia)
flywheel = chrono.ChBody()
flywheel.SetMass(10)  # kg
flywheel.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.2))  # Arbitrary inertia
flywheel.SetPos(chrono.ChVectorD(0.5, 0, 0))  # Centered at nodeC
sys.Add(flywheel)

# Link flywheel to the center node (nodeC)
link = fea.ChLinkNodeBody()
link.Initialize(nodeC, flywheel)
sys.Add(link)

# Create rotational motor at nodeA (driving end)
motor = chrono.ChLinkMotorRotationAngle()
# Align motor's rotation axis with global x-axis
motor_frame = chrono.ChFrameD(
    chrono.ChVectorD(0, 0, 0),
    chrono.Q_from_AngAxis(-chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))
)
motor.Initialize(sys.GetGround(), motor_frame, nodeA, chrono.ChFrameD())

# Set angular velocity (10 rad/s constant speed)
omega = 10.0
ramp = chrono.ChFunction_Ramp(0, omega)
motor.SetAngleFunction(ramp)
sys.Add(motor)

# Initialize Irrlicht visualization
vis = irr.ChIrrApp(sys, 'Jeffcott Rotor IGA Simulation', irr.dimension2du(1024, 768))
vis.AddTypicalLogo()
vis.AddTypicalCamera(irr.vector3df(0, 2, -3))  # Camera position
vis.AddTypicalLights()

# Add beam mesh to visualization with FEM displacement data
vis.AddMesh(mesh, 'Beam')
vis.GetMeshSceneNode(mesh).SetFEMdataType(chrono.ChIrrApp.FEM_DATA_DISPLACEMENT)
vis.GetMeshSceneNode(mesh).SetColor(chrono.ChColor(0.6, 0.6, 0.6))

# Finalize visualization setup
vis.AssetBindAll()
vis.AssetUpdateAll()

# Simulation loop
while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)  # Time step of 0.001 seconds