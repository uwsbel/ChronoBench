# PyChrono Jeffcott Rotor Simulation with IGA Beam

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np

# -----------------------------------------------------------------------------
# 1. Initialize the PyChrono environment and core components
# -----------------------------------------------------------------------------

# Create the physical system
sys = chrono.ChSystemSMC()

# Set gravity (point downward, along Y in this example)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create a FEM mesh to hold the IGA beam elements
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(True)

# -----------------------------------------------------------------------------
# 2. Define the beam material / section properties (IGA beam)
# -----------------------------------------------------------------------------

# Define the cross-section properties of the beam using an Euler-Bernoulli/
# advanced inertia section. Here we use ChBeamSectionCosserat-compatible model.

beam_wy = 0.012      # beam width (m)
beam_wz = 0.012      # beam height (m)

# Elasticity model for the cross-section
minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)              # steel density (kg/m^3)
minertia.SetArea(beam_wy * beam_wz)    # cross-sectional area
minertia.SetIyy((1.0 / 12.0) * beam_wz * beam_wy**3)
minertia.SetIzz((1.0 / 12.0) * beam_wy * beam_wz**3)

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)     # Young's modulus (Pa) for steel
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetArea(beam_wy * beam_wz)
melasticity.SetIyy((1.0 / 12.0) * beam_wz * beam_wy**3)
melasticity.SetIzz((1.0 / 12.0) * beam_wy * beam_wz**3)
melasticity.SetJ((1.0 / 12.0) * beam_wy * beam_wz * (beam_wy**2 + beam_wz**2))

# Combine into a Cosserat section
msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetDrawThickness(beam_wy, beam_wz)

# -----------------------------------------------------------------------------
# 3. Build the IGA beam using the builder helper
# -----------------------------------------------------------------------------

beam_length = 0.6     # total length of the rotor shaft (m)

builder = fea.ChBuilderBeamIGA()

# Build a straight IGA beam between two points
builder.BuildBeam(
    mesh,                                       # mesh to fill
    msection,                                   # section properties
    20,                                         # number of beam elements
    chrono.ChVector3d(0, 0, 0),                 # start point
    chrono.ChVector3d(beam_length, 0, 0),       # end point
    chrono.ChVector3d(0, 1, 0),                 # 'up' direction
    3                                           # polynomial order (cubic)
)

# Retrieve the nodes that were created
beam_nodes = builder.GetLastBeamNodes()

# Identify the start, center, and end nodes
node_start = beam_nodes[0]
node_end = beam_nodes[-1]
node_center = beam_nodes[len(beam_nodes) // 2]

# -----------------------------------------------------------------------------
# 4. Attach a flywheel (rigid body) at the center of the beam
# -----------------------------------------------------------------------------

flywheel = chrono.ChBody()
flywheel.SetMass(2.0)                            # flywheel mass (kg)
# Set inertia tensor (disk-like flywheel)
flywheel.SetInertiaXX(chrono.ChVector3d(0.02, 0.01, 0.02))
flywheel.SetPos(node_center.GetPos())
sys.Add(flywheel)

# Add a cylindrical visual shape to the flywheel
flywheel_shape = chrono.ChVisualShapeCylinder(0.08, 0.02)
flywheel.AddVisualShape(
    flywheel_shape,
    chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2))
)

# Connect the flywheel rigidly to the central beam node
constraint_flywheel = fea.ChLinkNodeFrame()
constraint_flywheel.Initialize(node_center, flywheel)
sys.Add(constraint_flywheel)

# Also constrain the rotation of the flywheel to follow the node frame
constraint_flywheel_rot = fea.ChLinkNodeSlopeFrame()
constraint_flywheel_rot.Initialize(node_center, flywheel)
constraint_flywheel_rot.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
sys.Add(constraint_flywheel_rot)

# -----------------------------------------------------------------------------
# 5. Create truss/ground bodies to support the beam ends
# -----------------------------------------------------------------------------

# Ground/truss body
truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)

# Bearing at the END node (free to rotate but constrained in position)
bearing_end = chrono.ChLinkMateGeneric()
bearing_end.Initialize(
    chrono.CastToChBodyFrame(chrono.CastToChBody(node_end)) if False else None,
    None, chrono.ChFramed()
) if False else None  # placeholder, replaced below

# Use a node-frame constraint to lock the end node position (a bearing)
constraint_end = fea.ChLinkNodeFrame()
constraint_end.Initialize(node_end, truss)
sys.Add(constraint_end)

# -----------------------------------------------------------------------------
# 6. Create a rotational motor that drives the START end of the beam
# -----------------------------------------------------------------------------

# Create a small stub body at the start to interface with the motor
stub_body = chrono.ChBody()
stub_body.SetMass(0.1)
stub_body.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))
stub_body.SetPos(node_start.GetPos())
sys.Add(stub_body)

# Attach the start node to the stub body
constraint_start = fea.ChLinkNodeFrame()
constraint_start.Initialize(node_start, stub_body)
sys.Add(constraint_start)

constraint_start_rot = fea.ChLinkNodeSlopeFrame()
constraint_start_rot.Initialize(node_start, stub_body)
constraint_start_rot.SetDirectionInBodyCoords(chrono.ChVector3d(1, 0, 0))
sys.Add(constraint_start_rot)

# Create the rotational motor between the truss and the stub body
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(
    stub_body,
    truss,
    chrono.ChFramed(node_start.GetPos(), chrono.QuatFromAngleY(chrono.CH_PI_2))
)

# Define a constant rotational speed (rad/s)
motor_speed = chrono.ChFunctionConst(20.0)   # ~191 rpm
motor.SetSpeedFunction(motor_speed)
sys.Add(motor)

# -----------------------------------------------------------------------------
# 7. Add the FEM mesh to the system
# -----------------------------------------------------------------------------

sys.Add(mesh)

# -----------------------------------------------------------------------------
# 8. FEM visualization setup
# -----------------------------------------------------------------------------

# Visualization of the beam: show bending moments mapped to colors
visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

# Visualization of the nodes as small coordinate systems
visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamB.SetSymbolsThickness(0.006)
visualizebeamB.SetSymbolsScale(0.01)
visualizebeamB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamB)

# -----------------------------------------------------------------------------
# 9. Create the Irrlicht visualization window
# -----------------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with IGA Beam')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.3, 0.3, -0.6),
              chrono.ChVector3d(0.3, 0.0, 0.0))
vis.AddTypicalLights()

# -----------------------------------------------------------------------------
# 10. Configure the solver and timestepper for stiff FEM dynamics
# -----------------------------------------------------------------------------

# Use a sparse direct solver for accuracy with FEM systems
solver = chrono.ChSolverPardisoMKL()
sys.SetSolver(solver)

# Use HHT implicit integrator for the stiff beam dynamics
stepper = chrono.ChTimestepperHHT(sys)
stepper.SetStepControl(False)
sys.SetTimestepper(stepper)

# -----------------------------------------------------------------------------
# 11. Run the simulation loop
# -----------------------------------------------------------------------------

time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)
```

## Key Components Explained

This script models a **Jeffcott rotor** with the following elements:

1. **IGA Beam**: Built with `ChBuilderBeamIGA` using a cubic B-spline beam discretized into 20 elements with a Cosserat cross-section (steel material properties).

2. **Flywheel**: A rigid cylindrical body (`ChBody`) attached at the **center** of the beam via `ChLinkNodeFrame` and `ChLinkNodeSlopeFrame` constraints, representing the rotating disk mass.

3. **Rotational Motor**: A `ChLinkMotorRotationSpeed` drives one end of the beam at a constant angular velocity (20 rad/s), simulating the spinning shaft.

4. **Bearings**: The two beam ends are connected to a fixed `truss` (the bearing supports).

5. **Visualization**: FEM visualization shows bending moments (`DataType_ELEM_BEAM_MZ`) as a color map, plus node coordinate systems, rendered in an Irrlicht window.

> **Note**: If `ChSolverPardisoMKL` is unavailable in your build, you can substitute `chrono.ChSolverSparseQR()` or the default solver. Tune the `motor_speed` to pass through the critical (resonant) speed to observe the characteristic Jeffcott whirling behavior.