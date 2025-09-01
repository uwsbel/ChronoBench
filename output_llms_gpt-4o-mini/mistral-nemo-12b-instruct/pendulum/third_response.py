import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the gravitational acceleration for the system (in m/s^2)
sys.Set_Gravitational_Acceleration(chrono.ChVector3d(0, -9.81, 0))  # g = 9.81 m/s^2

# Create the ground body and add it to the simulation
ground = chrono.ChBody()
sys.Add(ground)
ground.SetFixed(True)  # The ground is fixed and does not move
ground.Enable_Collision(False)  # Disable collision detection for the ground

# Add a visualization cylinder to the ground
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)  # Cylinder with radius 0.2 and height 0.4
ground.Add_Visual_Shape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, +1)))

# Create the first pendulum body and add it to the simulation
pend_1 = chrono.ChBody()
sys.Add_Body(pend_1)
pend_1.SetFixed(False)  # The pendulum can move
pend_1.Enable_Collision(False)  # Disable collision detection for the pendulum
pend_1.Set_Mass(1)  # Set the mass of the pendulum (in kg)
pend_1.Set_Inertia_XX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the first pendulum
cyl_pend_1 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_pend_1.Set_Color(chrono.ChColor(0.6, 0, 0))  # Set the color of the cylinder (RGB)
pend_1.Add_Visual_Shape(cyl_pend_1, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the first pendulum (center of mass) in the absolute frame
pend_1.Set_Pos(chrono.ChVector3d(1, 0, 1))

# Create a revolute joint to connect the first pendulum to the ground
rev_1 = chrono.ChLinkLockRevolute()
rev_1.Initialize(ground, pend_1, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add_Link(rev_1)

# Create the second pendulum body and add it to the simulation
pend_2 = chrono.ChBody()
sys.Add_Body(pend_2)
pend_2.SetFixed(False)  # The pendulum can move
pend_2.Enable_Collision(False)  # Disable collision detection for the pendulum
pend_2.Set_Mass(1)  # Set the mass of the pendulum (in kg)
pend_2.Set_Inertia_XX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia tensor (in kg·m^2)

# Add a visualization cylinder to the second pendulum
cyl_pend_2 = chrono.ChVisualShapeCylinder(0.2, 2)  # Cylinder with radius 0.2 and height 2
cyl_pend_2.Set_Color(chrono.ChColor(0, 0, 0.6))  # Set the color of the cylinder (RGB)
pend_2.Add_Visual_Shape(cyl_pend_2, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))

# Set the initial position of the second pendulum (center of mass) in the absolute frame
pend_2.Set_Pos(chrono.ChVector3d(2, 0, 1))

# Create a revolute joint to connect the second pendulum to the first pendulum
rev_2 = chrono.ChLinkLockRevolute()
rev_2.Initialize(pend_1, pend_2, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
sys.Add_Link(rev_2)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.Attach_System(sys)  # Attach the Chrono system to the visualization
vis.Set_Window_Size(1024, 768)  # Set the window size for the visualization
vis.Set_Window_Title('Double Pendulum demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.Add_Logo(chrono.Get_Chrono_Data_File('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.Add_SkyBox()  # Add a skybox for better visual appearance
vis.Add_Camera(chrono.ChVector3d(0, 3, 6))  # Add a camera to the visualization
vis.Add_Typical_Lights()  # Add typical lights for better visualization

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.Begin_Scene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.End_Scene()  # End the scene
    sys.Do_Step_Dynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds

    # Log information after 1 second of simulation time
    if log_info and sys.Get_Ch_Time() > 1:
        pos_1 = pend_1.Get_Pos()  # Get the position of the first pendulum
        pos_2 = pend_2.Get_Pos()  # Get the position of the second pendulum
        print("t = ", sys.Get_Ch_Time())
        print("     ", pos_1.x, "  ", pos_1.y)
        print("     ", pos_2.x, "  ", pos_2.y)
        log_info = False  # Disable further logging