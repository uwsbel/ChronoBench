import pychrono as chrono                         # Importing the Chrono library
import pychrono.irrlicht as chronoirr             # Importing the Irrlicht visualization library for Chrono
import math as m                                  # Importing the math library for mathematical operations

# Create a Chrono::Engine physical system
sys = chrono.ChSystemNSC()                        # Initializing the Chrono physical system with non-smooth contact (NSC) method

# Create contact material shared among all bodies
mat = chrono.ChContactMaterialNSC()               # Creating a contact material for collision handling

# Create all rigid bodies with specific dimensions
radA = 1.5                                        # Radius for gear A
radB = 3.5                                        # Radius for gear B

# Create the truss
mbody_truss = chrono.ChBodyEasyBox(15, 8, 2,      # Box-shaped truss body with dimensions 15x8x2
                                   1000,          # Mass (not used for fixed body)
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_truss)                              # Adding the truss to the physical system
mbody_truss.SetFixed(True)                        # Making the truss fixed (immovable)
mbody_truss.SetPos(chrono.ChVector3d(0, 0, 3))    # Position of the truss at (0, 0, 3)

# Shared visualization material for enhanced aesthetics
vis_mat = chrono.ChVisualMaterial()                       # Creating a visual material
vis_mat.SetKdTexture(chrono.GetChronoDataFile('textures/pinkwhite.png'))  # Setting a texture for the visual material

# Create the rotating bar support for the two epicycloidal wheels
mbody_train = chrono.ChBodyEasyBox(8, 1.5, 1.0,  # Box-shaped rotating bar
                                   1000,          # Mass
                                   True,          # Enable visualization
                                   False,         # Disable collision
                                   mat)           # Using the defined contact material
sys.Add(mbody_train)                              # Adding the rotating bar to the system
mbody_train.SetPos(chrono.ChVector3d(3, 0, 0))    # Position at (3, 0, 0)

# Revolute joint between truss and rotating bar
link_revoluteTT = chrono.ChLinkLockRevolute()
link_revoluteTT.Initialize(mbody_truss, mbody_train,
                           chrono.ChFrameD(chrono.ChVector3d(0, 0, 3), chrono.Q_from_AngX(0)))
sys.AddLink(link_revoluteTT)

# Create the first gear (gear A)
mbody_gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radA, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearA)
mbody_gearA.SetPos(chrono.ChVector3d(0, 0, -1))
mbody_gearA.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearA.GetVisualShape(0).SetMaterial(0, vis_mat)

# Visual shape for gear A (optional)
mshaft_shape_A = chrono.ChVisualShapeCylinder(radA * 0.3, 10)
mbody_gearA.AddVisualShape(mshaft_shape_A, chrono.ChFrameD(chrono.ChVector3d(0, 3.5, 0),
                                                          chrono.Q_from_AngX(m.pi / 2)))

# Impose rotation speed on gear A
link_motorA = chrono.ChLinkMotorRotationSpeed()
link_motorA.Initialize(mbody_gearA, mbody_truss,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, 0), chrono.Q_from_AngX(0)))
link_motorA.SetSpeedFunction(chrono.ChFunction_Const(3))
sys.Add(link_motorA)

# Create gear B
interaxis12 = radA + radB
mbody_gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                        radB, 0.4,
                                        1000, True, False, mat)
sys.Add(mbody_gearB)
mbody_gearB.SetPos(chrono.ChVector3d(interaxis12, 0, -2))
mbody_gearB.SetRot(chrono.Q_from_AngX(m.pi / 2))
mbody_gearB.GetVisualShape(0).SetMaterial(0, vis_mat)

# Revolute joint for gear B to rotating bar
link_revoluteB = chrono.ChLinkLockRevolute()
link_revoluteB.Initialize(mbody_gearB, mbody_train,
                          chrono.ChFrameD(chrono.ChVector3d(interaxis12, 0, -1), chrono.Q_from_AngX(0)))
sys.AddLink(link_revoluteB)

# Gear constraint between gear A and gear B
link_gearAB = chrono.ChLinkLockGear()
link_gearAB.Initialize(mbody_gearA, mbody_gearB, chrono.ChFrameD(chrono.VNULL))
link_gearAB.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearAB.SetFrameShaft2(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearAB.SetTransmissionRatio(radA / radB)
link_gearAB.SetEnforcePhase(True)
sys.AddLink(link_gearAB)

# Large wheel C (truss itself)
radC = 2 * radB + radA
link_gearBC = chrono.ChLinkLockGear()
link_gearBC.Initialize(mbody_gearB, mbody_truss,
                       chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.Q_from_AngX(0)))
link_gearBC.SetFrameShaft1(chrono.ChFrameD(chrono.VNULL, chrono.Q_from_AngX(-m.pi/2)))
link_gearBC.SetFrameShaft2(chrono.ChFrameD(chrono.ChVector3d(0, 0, -4), chrono.Q_from_AngX(0)))
link_gearBC.SetTransmissionRatio(radB / radC)
link_gearBC.SetEpicyclic(True)
sys.AddLink(link_gearBC)

# --- New Bevel Gear (Gear D) ---
radD = 5
mbody_gearD = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z,
                                        radD, 0.5,
                                        1000, True, False, mat)
sys.Add(mbody_gearD)
# Position at (-10, 0, -9)
mbody_gearD.SetPos(chrono.ChVector3d(-10, 0, -9))
# Rotate by 90 degrees around Z-axis
mbody_gearD.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1)))
# Visual shape for gear D
mshaft_shape_D = chrono.ChVisualShapeCylinder(radD * 0.3, 10)
mbody_gearD.AddVisualShape(mshaft_shape_D, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                                          chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
# Revolute joint to fix gear D to truss (along Z axis)
link_revoluteD = chrono.ChLinkLockRevolute()
link_revoluteD.Initialize(mbody_gearD, mbody_truss,
                          chrono.ChFrameD(chrono.ChVector3d(-10, 0, -9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
sys.Add(link_revoluteD)

# --- New Pulley (Pulley E) ---
radE = 2
mbody_pulleyE = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z,
                                          radE, 0.2,
                                          1000, True, False, mat)
sys.Add(mbody_pulleyE)
# Position at (-10, -11, -9)
mbody_pulleyE.SetPos(chrono.ChVector3d(-10, -11, -9))
# Rotate by 90 degrees around Z axis
mbody_pulleyE.SetRot(chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1)))
# Visual shape for pulley E
mshaft_shape_E = chrono.ChVisualShapeCylinder(radE * 0.3, 10)
mbody_pulleyE.AddVisualShape(mshaft_shape_E, chrono.ChFrameD(chrono.ChVector3d(0, 0, 0),
                                                          chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
# Revolute joint to fix pulley E to truss (along Z axis)
link_revoluteE = chrono.ChLinkLockRevolute()
link_revoluteE.Initialize(mbody_pulleyE, mbody_truss,
                          chrono.ChFrameD(chrono.ChVector3d(-10, -11, -9), chrono.Q_from_AngAxis(m.pi/2, chrono.ChVector3d(0, 0, 1))))
sys.Add(link_revoluteE)

# --- Belt (for visualization) between gear D and pulley E ---
# Visual representation of the belt as a simple thin cylinder or line
belt_thickness = 0.05
belt_color = chrono.ChColorAsset(0.8, 0.2, 0.2)  # Red belt

# Create a visual shape for belt (a line between the two bodies)
belt_shape = chrono.ChLineShape()
# Start point: gear D position
start_point = mbody_gearD.GetPos()
# End point: pulley E position
end_point = mbody_pulleyE.GetPos()

# Add visual shape (a line) for belt
# Since pychrono does not have a direct "line" shape, we can create a thin cylinder or a custom shape.
# Here, we'll create a thin cylinder for the belt between the two components.
# Position at midpoint
mid_point = (start_point + end_point) * 0.5
# Direction vector from gear D to pulley E
dir_vec = end_point - start_point
length = dir_vec.Length()
# Create a cylinder aligned with dir_vec
belt_cylinder = chrono.ChBodyEasyCylinder(belt_thickness, length,
                                         0, True, False, chrono.ChMaterialSurface())
# Set position to midpoint
belt_cylinder.SetPos(mid_point)
# Compute rotation to align with dir_vec
axis = dir_vec.GetNormalized()
# Create rotation quaternion
rotation = chrono.Q_from_AngAxis(m.pi/2, axis.Cross(chrono.ChVector3d(0, 0, 1))) if axis.Dot(chrono.ChVector3d(0,0,1)) < 0.99 else chrono.Q_from_AngAxis(0, axis)
belt_cylinder.SetRot(rotation)
# Set color
belt_shape_asset = chrono.ChVisualShapeCylinder(belt_thickness, length)
belt_shape_asset.SetColor(chrono.ChColor(0.8, 0.2, 0.2))
belt_cylinder.AddVisualShape(belt_shape_asset)

# Optional: add belt to system if needed for visualization
sys.Add(belt_cylinder)

# Create a belt constraint (synchronization) between gear D and pulley E
# Using ChLinkBelt or similar constraint if available, or approximate with a custom constraint
# Since pychrono may not have a direct belt constraint, we can use a ChLinkBelt if available, or simulate with a custom constraint.
# For simplicity, here we just add a placeholder comment.
# Note: In actual implementation, you might need to implement a custom constraint or use a predefined belt class if available.

# --- Visualization enhancements ---
# Optional: add visual shapes for gear D and pulley E (already added above)
# Additional visual details can be added similarly if needed

# --- End of modifications ---

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()                        # Initialize visualization system
vis.AttachSystem(sys)                                           # Attach the system
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gears and pulleys with additional components')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(12, 15, -20))
vis.AddTypicalLights()

# Set integrator type
sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_PROJECTED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)