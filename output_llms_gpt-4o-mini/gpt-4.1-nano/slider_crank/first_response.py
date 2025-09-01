import pychrono.core as chrono

# Initialize the Chrono system
system = chrono.ChSystemNSC()

# --- Add Floor (Static Truss)
floor = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
floor.SetPosition(chrono.ChVectorD(0, -0.25, 0))
floor.SetBodyFixed(True)
system.Add(floor)

# --- Add Crankshaft
crank_radius = 0.5
crank_length = 0.1
crank_mass = 1.0

crank = chrono.ChBodyEasyCylinder(crank_radius, crank_length, crank_mass, True, True)
crank.SetMass(crank_mass)
crank.SetPos(chrono.ChVectorD(0, 0.0, 0))
system.Add(crank)

# Position the crank so that its rotation axis is at the origin
crank.SetIdentifier(1)

# --- Add Connecting Rod
rod_length = 2.0
rod_radius = 0.05
rod_mass = 0.5
connecting_rod = chrono.ChBodyEasyCylinder(rod_radius, rod_length, rod_mass, True, True)
connecting_rod.SetMass(rod_mass)
connecting_rod.SetPos(chrono.ChVectorD(rod_length / 2, 0, 0))
system.Add(connecting_rod)

# --- Add Piston
piston_radius = 0.3
piston_height = 0.2
piston_mass = 1.0
piston = chrono.ChBodyEasyCylinder(piston_radius, piston_height, piston_mass, True, True)
piston.SetMass(piston_mass)
piston.SetPos(chrono.ChVectorD(rod_length + 0.5, 0, 0))
system.Add(piston)

# --- Connect Crankshaft to Floor with a Revolution Joint (revolute)
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crank, floor, chrono.ChVectorD(0, 0, 0))
system.Add(crank_joint)

# --- Add a motor to drive the crankshaft at constant speed
motor_speed = 10  # rad/s (can be adjusted)

crank_motor = chrono.ChLinkMotorSpeed()
crank_motor.Initialize(crank, floor, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
crank_motor.SetSpeed(motor_speed)
system.Add(crank_motor)

# --- Connect Connecting Rod to Crankshaft with a Revolute joint
# Attach one end of the rod to the crank
rod_crank_joint = chrono.ChLinkLockRevolute()
rod_crank_joint.Initialize(connecting_rod, crank, chrono.ChVectorD(0, 0, 0))
system.Add(rod_crank_joint)

# --- Connect Connecting Rod to Piston with a Revolute joint
# Attach other end of the rod to the piston
# Position the connecting point on the rod at its end
rod_end_point = chrono.ChVectorD(rod_length, 0, 0)

rod_piston_joint = chrono.ChLinkLockRevolute()
rod_piston_joint.Initialize(connecting_rod, piston, roe_piston_point := chrono.ChFrameD(chrono.ChVectorD(rod_length, 0, 0)))
system.Add(rod_piston_joint)

# --- To constrain piston movement to vertical (Y-axis), use a prismatic joint
piston_constraint = chrono.ChLinkLockPrismatic()
piston_constraint.Initialize(piston, floor, chrono.ChFrameD(chrono.ChVectorD(rod_length + 0.5, 0, 0), chrono.Q_from_YOY(1, 0, 0)))
system.Add(piston_constraint)

# Alternatively, restrict piston to move only in Y with a linear motor or limit if needed
# For simplicity, here we assume vertical movement along Y axis

# --- Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# --- Visualization: Irrlicht setup
visualization = chrono.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(1280, 720)
visualization.SetWindowTitle('Crank-Slider Mechanism Simulation')

# Camera positioning
visualization.SetCamera(chrono.ChVectorD(-5, 2, 10),  # camera position
                        chrono.ChVectorD(0, 0, 0))   # look-at point

# Enable lighting
visualization.EnableLighting(True)
visualization.AddLights()
visualization.LightsBegin()
visualization.LightsSetBack(chrono.ChColor(0.2, 0.2, 0.2))
visualization.LightsSetFront(chrono.ChColor(0.8, 0.8, 0.8))
visualization.LightsEnd()

# Optional: Add background color or textures (if desired)
# visualization.SetBackgroundColor(chrono.ChColor(0.1, 0.1, 0.2))
# visualization.AddLogo("path_to_logo.png", 0.5, 0.5, 0.2)

# --- Run the simulation
dt = 0.005  # time step
while visualization.Run():
    system.DoStepDynamics(dt)