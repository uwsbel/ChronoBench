import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2

# Parameters for the pendulums
# Pendulum 1
L1 = 2.0  # Length
R1 = 0.1  # Radius
M1 = 1.0  # Mass

# Pendulum 2
L2 = 1.5  # Length
R2 = 0.08 # Radius
M2 = 0.8  # Mass

# Joint Z-coordinate (all joints will be at this Z level)
JOINT_Z = 1.0

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.EnableCollision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to mark the first pivot on the ground
ground_pivot_vis = chrono.ChVisualShapeCylinder(0.15, 0.4)  # Cylinder with radius 0.15 and height 0.4
ground_pivot_vis.SetColor(chrono.ChColor(0.3, 0.3, 0.3)) # Grey color
ground.AddVisualShape(ground_pivot_vis, chrono.ChFramed(chrono.ChVector3d(0, 0, JOINT_Z))) # Centered at the pivot

# --- Pendulum 1 ---
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)
pend_1.SetMass(M1)

# Calculate inertia for pendulum 1 (cylinder)
inertia_p1_x = 0.5 * M1 * R1**2
inertia_p1_yz = (1/12) * M1 * (3 * R1**2 + L1**2)
pend_1.SetInertiaXX(chrono.ChVector3d(inertia_p1_x, inertia_p1_yz, inertia_p1_yz))

# Add a visualization cylinder to pendulum 1
# The cylinder's length is L1, its axis is aligned with the body's local X-axis.
vis_shape_p1 = chrono.ChVisualShapeCylinder(R1, L1)
vis_shape_p1.SetColor(chrono.ChColor(0.8, 0.2, 0.2))  # Red color
# Rotate the visual shape so its length (height of cylinder) is along body's X axis
pend_1.AddVisualShape(vis_shape_p1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Initial position of pendulum 1 (CoM) - horizontal, along +X
# Pivot 1 is at (0, 0, JOINT_Z)
pos_p1_com = chrono.ChVector3d(L1/2, 0, JOINT_Z)
pend_1.SetPos(pos_p1_com)
pend_1.SetRot(chrono.QUNIT) # Initial rotation: body X along global X

# Create revolute joint between ground and pendulum 1
rev_1 = chrono.ChLinkLockRevolute()
# Joint frame in absolute coordinates: at (0,0,JOINT_Z), Z-axis is rotation axis
joint_pos_1 = chrono.ChVector3d(0, 0, JOINT_Z)
rev_1.Initialize(ground, pend_1, chrono.ChFramed(joint_pos_1, chrono.QUNIT))
sys.AddLink(rev_1)


# --- Pendulum 2 ---
pend_2 = chrono.ChBody()
sys.AddBody(pend_2)
pend_2.SetFixed(False)
pend_2.EnableCollision(False)
pend_2.SetMass(M2)

# Calculate inertia for pendulum 2 (cylinder)
inertia_p2_x = 0.5 * M2 * R2**2
inertia_p2_yz = (1/12) * M2 * (3 * R2**2 + L2**2)
pend_2.SetInertiaXX(chrono.ChVector3d(inertia_p2_x, inertia_p2_yz, inertia_p2_yz))

# Add a visualization cylinder to pendulum 2
vis_shape_p2 = chrono.ChVisualShapeCylinder(R2, L2)
vis_shape_p2.SetColor(chrono.ChColor(0.2, 0.2, 0.8))  # Blue color
# Rotate the visual shape so its length (height of cylinder) is along body's X axis
pend_2.AddVisualShape(vis_shape_p2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Initial position of pendulum 2 (CoM) - horizontal, along +X, attached to end of pend_1
# Pivot 2 is at end of pend_1: (L1, 0, JOINT_Z)
pos_p2_com = chrono.ChVector3d(L1 + L2/2, 0, JOINT_Z)
pend_2.SetPos(pos_p2_com)
pend_2.SetRot(chrono.QUNIT) # Initial rotation: body X along global X

# Create revolute joint between pendulum 1 and pendulum 2
rev_2 = chrono.ChLinkLockRevolute()
# Joint frame in absolute coordinates: at (L1,0,JOINT_Z), Z-axis is rotation axis
joint_pos_2 = chrono.ChVector3d(L1, 0, JOINT_Z)
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(joint_pos_2, chrono.QUNIT))
sys.AddLink(rev_2)


# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Double Pendulum Demo') # Corrected title
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(L1, L1, JOINT_Z + L1*1.5)) # Adjusted camera for better view
vis.SetCameraTarget(chrono.ChVector3d(L1/2, 0, JOINT_Z)) # Camera target point
vis.AddTypicalLights()

# Simulation loop
log_info = True
time_step = 1e-3

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)

    if log_info and sys.GetChTime() > 1:
        pos_p2 = pend_2.GetPos()
        print(f"t = {sys.GetChTime():.3f} s")
        print(f"  Pendulum 2 COM Pos: x={pos_p2.x:.3f}, y={pos_p2.y:.3f}, z={pos_p2.z:.3f}")
        vel_p2 = pend_2.GetPosDt()
        print(f"  Pendulum 2 COM Vel: vx={vel_p2.x:.3f}, vy={vel_p2.y:.3f}, vz={vel_p2.z:.3f}")
        
        # Optional: Log pend_1 info too
        # pos_p1 = pend_1.GetPos()
        # print(f"  Pendulum 1 COM Pos: x={pos_p1.x:.3f}, y={pos_p1.y:.3f}, z={pos_p1.z:.3f}")
        
        log_info = False # Disable further logging after the first time

del sys