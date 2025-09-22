import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# -----------------------------------------------------------------------------
# 1) Create the Chrono system
# -----------------------------------------------------------------------------
chrono.SetChronoDataPath("")      # not really needed here, but good practice
sys = chrono.ChSystemNSC()        # non–smooth (default) NSC system

# -----------------------------------------------------------------------------
# 2) Create the ground body (fixed)
# -----------------------------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)

# -----------------------------------------------------------------------------
# 3) Create the mass body
# -----------------------------------------------------------------------------
mass = chrono.ChBody()
mass.SetMass(1.0)                        # 1 kg
mass.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))  # simple diagonal inertia
mass.SetPos(chrono.ChVectorD(0, 0, -1.0))            # start 1 m below ground

# Visual: a small blue sphere
sphere_shape = chrono.ChSphereShape()
sphere_shape.GetSphereGeometry().rad = 0.1
mass.AddAsset(sphere_shape)

color_asset = chrono.ChColorAsset()
color_asset.SetColor(chrono.ChColor(0.2, 0.2, 1.0))   # blue
mass.AddAsset(color_asset)

sys.Add(mass)

# -----------------------------------------------------------------------------
# 4) Create the spring–damper (ChLinkTSDA)
# -----------------------------------------------------------------------------
spring = chrono.ChLinkTSDA()

# Define the two attachment points in each body's LOCAL frame.
# Here we attach to ground at (0,0,0) and to the mass at (0,0,0) in the mass's frame.
pA = chrono.ChVectorD(0, 0, 0)      # point on ground
pB = chrono.ChVectorD(0, 0, 0)      # point on mass

# Initialize the TSDA between ground and mass:
spring.Initialize(ground,            # body A
                  mass,              # body B
                  False,             # no auto-align
                  chrono.ChVectorD(0,0,0),  # marker on A in global coords
                  chrono.ChVectorD(0,0,-1)) # marker on B in global coords

# Set linear spring–damper parameters:
spring.SetSpringCoefficient(50.0)   # N/m
spring.SetDampingCoefficient(5.0)   # N·s/m
spring.SetRestLength(1.0)            # natural length = 1 m

sys.Add(spring)

# -----------------------------------------------------------------------------
# 5) Add a visual cylinder to represent the spring
# -----------------------------------------------------------------------------
cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().rad = 0.02   # small radius
# set an initial pair of endpoints (will be updated every frame):
cyl.SetEndPoints(chrono.ChVectorD(0,0,0), chrono.ChVectorD(0,0,-1))
spring.AddAsset(cyl)

# -----------------------------------------------------------------------------
# 6) Set up the Irrlicht visualization
# -----------------------------------------------------------------------------
app = chronoirr.ChIrrApp(sys,
                         "Mass–Spring–Damper Demo",
                         chronoirr.dimension2du(1024, 768))

# typical camera, lights and skybox
app.AddLogo()                  # Chrono logo
app.AddSkyBox()
app.AddTypicalLights()
app.AddTypicalCamera(chronoirr.vector3df(1,1,1),  # cam pos
                     chronoirr.vector3df(0,0,0))  # look-at

# bind & update assets to Irrlicht
app.AssetBindAll()
app.AssetUpdateAll()

# -----------------------------------------------------------------------------
# 7) Run the simulation
# -----------------------------------------------------------------------------
t_end = 5.0
dt    = 1e-3

while app.Run():
    app.BeginScene()
    app.DrawAll()
    # update the spring‐cylinder endpoints
    pA_w = spring.GetEndPoint1World()
    pB_w = spring.GetEndPoint2World()
    cyl.SetEndPoints(pA_w, pB_w)
    app.EndScene()
    sys.DoStepDynamics(dt)