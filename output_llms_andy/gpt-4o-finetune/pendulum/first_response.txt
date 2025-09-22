import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# Initialize the Chrono simulation system
sys = chrono.ChSystemNSC()

# Set the system's collision detection method
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Define the default material properties for contact interactions
mat = chrono.ChContactMaterialNSC()
mat.SetFriction(0.5)  # Set the friction coefficient

# Create and configure the ground body
ground = chrono.ChBody()
sys.AddBody(ground)  # Add the ground to the simulation system
ground.SetFixed(True)  # Fix the ground in place
ground.SetPos(chrono.ChVector3d(0, 0, 0))  # Position the ground
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # Set ground texture

# Create a box shape for the ground's visualization
box = chrono.ChVisualShapeBox(20, 20, 1)  # Define a box shape
box.SetColor(chrono.ChColor(0.6, 0, 0))  # Set the box color
ground.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(0, 0, -1)))  # Add the visual shape to the ground

# Create and configure the pendulum body
pend = chrono.ChBody()
sys.AddBody(pend)  # Add the pendulum to the simulation system
pend.SetPos(chrono.ChVector3d(0, -1, 0))  # Position the pendulum
pend.SetFixed(False)  # Ensure the pendulum is not fixed
pend.SetMass(1)  # Set the mass of the pendulum
pend.SetInertiaXX(chrono.ChVector3d(0.2, 1, 1))  # Set the inertia of the pendulum
pend.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # Set pendulum texture

# Create a box shape for the pendulum's visualization
box = chrono.ChVisualShapeBox(1, 2, 1)  # Define a box shape
box.SetColor(chrono.ChColor(0, 0, 0.6))  # Set the box color
pend.AddVisualShape(box)  # Add the visual shape to the pendulum

# Create and configure the revolute joint (hinge) between the ground and pendulum
rev_joint = chrono.ChJointRevolute()
rev_joint.Initialize(ground, pend, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))  # Initialize the joint
sys.AddLink(rev_joint)  # Add the joint to the system
# Optionally, add a visual representation of the joint
# rev_joint.GetVisualShapeForLink().SetThickness(0.05)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  # Attach the Chrono system to the visualization
vis.SetWindowSize(1024, 768)  # Set the window size for the visualization
vis.SetWindowTitle('ChBodyAuxRef demo')  # Set the window title
vis.Initialize()  # Initialize the visualization system
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))  # Add a logo to the visualization
vis.AddSkyBox()  # Add a skybox for better visuals
vis.AddCamera(chrono.ChVector3d(0, 5, -6))  # Add and position the camera
vis.AddTypicalLights()  # Add typical lights for the scene

# Simulation loop
log_info = True  # Flag to control logging
while vis.Run():
    vis.BeginScene()  # Begin the scene for rendering
    vis.Render()  # Render the scene
    vis.EndScene()  # End the scene
    
    if log_info:
        # Log information about the pendulum
        pos = pend.GetPos()  # Get the pendulum's position
        lin_vel = pend.GetPosDt()  # Get the pendulum's linear velocity
        print("     t = 0.0s x = " + str(round(pos.x, 3)) + "  y = " + str(round(pos.y, 3)) + "  z = " + str(
            round(pos.z, 3)) + "  Vx = " + str(round(lin_vel.x, 3)) + "  Vy = " + str(round(lin_vel.y, 3)) + "  Vz = " + str(
            round(lin_vel.z, 3)))
        log_info = False  # Disable further logging
    
    sys.DoStepDynamics(1e-3)  # Advance the simulation by a time step of 0.001 seconds