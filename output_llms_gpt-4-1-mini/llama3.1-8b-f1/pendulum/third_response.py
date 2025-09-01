import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change the mass of the pendulum bodies during simulation.
# This is an example of using callbacks to change the simulation dynamics at runtime.
change_mass = False

# Initial position of the pendulum
initLoc = chrono.ChVector3d(0, 0, 1.2)

# Initial swing-up method
#   0: no swing up
#   1: simple pivot point swing up
#   2: no swing up (just release from rest)
swingUpMethod = 1

# Pivot point method
#   0: no pivot point
#   1: pivot point at base of pendulum
#   2: pivot point at mid-length of pendulum
#   3: pivot point at 2/3 length of pendulum
pivotMethod = 1

# Collision system type
#   0: None
#   1: Bullet
#   2: NSC
#   3: SMC
collisionType = 2

# ----------------------------------------------
# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
# ----------------------------------------------

print( "Copyright (c) 2017 projectchrono.org\n")

# Create the pendulum system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(collisionType)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Create the first pendulum body
pend1 = chrono.ChBody()
pend1.SetMass(1)
pend1.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(pend1)

# Create the second pendulum body
pend2 = chrono.ChBody()
pend2.SetMass(1)
pend2.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
sys.Add(pend2)

# Create the visualization shapes for the pendulum bodies
ball = chrono.ChVisualShapeSphere(0.1)
ball.SetTexture(chrono.GetChronoDataFile("textures/checker2.png"))
pend1.AddVisualShape(ball)
pend2.AddVisualShape(ball)

# Create the link between the pendulum bodies
link1 = chrono.ChLinkLockRevolute()
link1.Initialize(pend1, pend2, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0)))
sys.AddLink(link1)

link2 = chrono.ChLinkLockRevolute()
if (pivotMethod == 1):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleZ(0)))
elif (pivotMethod == 2):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5, 0, 0, 0), chrono.QuatFromAngleZ(0)))
elif (pivotMethod == 3):
    link2.Initialize(pend2, ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 0.67, 0, 0, 0), chrono.QuatFromAngleZ(0)))
sys.AddLink(link2)

# Set the initial position of the pendulum
pend1.SetPos(chrono.ChVector3d(initLoc.x, initLoc.y, initLoc.z))
pend2.SetPos(chrono.ChVector3d(0, 0, 0))

# Enable dynamics for the pendulum bodies
pend1.EnableCollision(True)
pend2.EnableCollision(True)

# Set the collision system type for the pendulum bodies
if (collisionType == 1):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
elif (collisionType == 2):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSC)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_NSC)
elif (collisionType == 3):
    pend1.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)
    pend2.GetCollisionModel().SetCollisionSystemType(chrono.ChCollisionSystem.Type_SMC)

# Create and initialize the irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Double Pendulum')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2,2,3))
vis.AddTypicalLights()

# Create the driver system
driver = chronoirr.ChDriverIRR()
driver.SetChTime(0.0)
driver.SetChStepSize(1e-3)
driver.SetChMaxSteps(1000)
driver.SetWindowSize(1024,768)
driver.SetWindowTitle('Double Pendulum')
driver.Initialize()
driver.AddTypicalLights()
driver.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddSkyBox()
driver.AddCamera(chrono.ChVector3d(2,2,3))
driver.AddTypicalOrbitCamera(chrono.ChVector3d(2,2,3),0.5,100.0)
driver.AddLightWithShadow(chrono.ChVector3d(2,10,5),chrono.ChVector3d(0,0,0),50,100.0,512)

# Set the time response for mouse clicks to change the lengths of pendulum links.
# Note that this is different from the typical time response for camera rotations/orbitals,
# which is set in driver.AddTypicalOrbitCamera() above.
driver.SetMouseTimeResponse(0.2)

# Initialize driver system mode
# Note that this demo uses driver primarily for getting time from the computer, 
# and for visualization of the system. The actual dynamics is computed by the system itself.
driver.SetDriverSystemMode(driver.DRIVERSYS_MODE_GETTIME | driver.DRIVERSYS_MODE_VISUALIZATION | driver.DRIVERSYS_MODE_MOUSE)

# Set the simulation step sizes
pend1.SetStepSize(1e-3)
pend2.SetStepSize(1e-3)

# Initialize the pendulum angles (for both bodies)
# These are used to initialize the visualization
ang1 = 0
ang2 = 0

# ----------------------------------------------
# Generate PLOT data for external plotting programs
# ----------------------------------------------
#
# If true, generate data for plotting (for example for gnuplot).
# The file format is CSV (comma separated values).
plot = False
#
# If plot is true, then generate data for the pendulum at the given path.
# This is a list of (x,y,z) coordinates that the pendulum will follow.
path = [
    chrono.ChVector3d(-2, 0, 1.2),
    chrono.ChVector3d(-1.5, -0.5, 1.1),
    chrono.ChVector3d(-1, -1, 1.0),
    chrono.ChVector3d(-0.5, -1.5, 0.9),
    chrono.ChVector3d(0, -2, 0.8),
    chrono.ChVector3d(0.5, -2.5, 0.7),
    chrono.ChVector3d(1, -3, 0.6),
    chrono.ChVector3d(1.5, -3.5, 0.5),
    chrono.ChVector3d(2, -4, 0.4),
    chrono.ChVector3d(2.5, -4.5, 0.3),
    chrono.ChVector3d(3, -5, 0.2),
    chrono.ChVector3d(3.5, -5.5, 0.1),
    chrono.ChVector3d(4, -6, 0),
    ]
plot_path = True

# -----------------
# End of parameters
# -----------------

print ("Double Pendulum Demo\n")

# Initialize pendulum visualization angles
vis1 = 0
vis2 = 0

# Create a callback function to modify the mass of the pendulum bodies during simulation.
# This is an example of using callbacks to change the simulation dynamics at runtime.
def change_mass_callback(mass):
    pend1.SetMass(mass)
    pend2.SetMass(mass)
    print('Mass changed to ' + str(mass) + '\n')
    return mass * 1.1

# Change the mass of the pendulum bodies during simulation.
if (change_mass):
    change_mass_callback_data = [1.0, 0.5]

# Simulation loop
time = 0
time_end = 100

while (time < time_end):

    # Get the current time
    time = sys.GetChTime()

    # End simulation
    if (time >= time_end):
        break

    # Draw scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Get the current angles of the pendulum from the visualization system
    vis1 = pend1.GetAngleY() * (180.0 / chrono.CH_PI)
    vis2 = pend2.GetAngleY() * (180.0 / chrono.CH_PI)

    # Print the current angles of the pendulum
    print("Time = " + str(time) + "  |ang1| = " + str(abs(vis1)) + "  |ang2| = " + str(abs(vis2)) + "\n")

    # Check for invalid pendulum angles (if so, abort)
    if (abs(vis1) >= 100):
        break

    # Change the length of the pendulum links based on the current mouse position
    # (only if the driver system has rendered the scene)
    if (driver.HasRendered()) :
        mposx = driver.GetMouseX()
        mposy = driver.GetMouseY()
        ratio = (mposx - 0.5) * 0.2
        if (link1.GetA() > 0):
            link1.SetA(2 + ratio)
            link2.SetA(2 + ratio)
        vis.Synchronize()
        driver.Synchronize()

    # Advance the state of the entire system for one timestep for all bodies
    sys.DoStepDynamics(0.0025)

# Return from main demo function
return