import pychrono as chrono
import pychrono.irrlicht as chronoirr

# -------------------------------------------------------
# 1) Initialize Chrono system
# -------------------------------------------------------
chrono.SetChronoDataPath('.')     # adjust if you have a custom data folder
system = chrono.ChSystemNSC()

# -------------------------------------------------------
# 2) Create a fixed ground (a simple cylinder as "truss")
# -------------------------------------------------------
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
# visual asset (cylinder along X, length 2, radius 0.05)
cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-1, 0, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD( 1, 0, 0)
cyl.GetCylinderGeometry().rad = 0.05
ground.AddAsset(cyl)
# gray color
ground.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.5, 0.5, 0.5)))
system.Add(ground)

# -------------------------------------------------------
# 3) Create the rotating bar (pinned at origin)
# -------------------------------------------------------
bar = chrono.ChBody()
bar.SetMass(2.0)
# simple inertia guess
bar.SetInertiaXX(chrono.ChVectorD(0.02, 0.02, 0.02))
bar.SetPos(chrono.ChVectorD(0, 0, 0))
# visual cylinder along X, half‐length 0.5, radius 0.03
bar_cyl = chrono.ChCylinderShape()
bar_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.5, 0, 0)
bar_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD( 0.5, 0, 0)
bar_cyl.GetCylinderGeometry().rad = 0.03
bar.AddAsset(bar_cyl)
bar.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.4, 0.8)))
system.Add(bar)

# Revolute joint bar <-> ground about Z axis at origin
rev_bar = chrono.ChLinkLockRevolute()
rev_bar.Initialize(bar, ground,
                   chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                                   chrono.Q_from_AngAxis(0, 0, 1, 0)))
system.AddLink(rev_bar)

# Motor: constant angular speed = 1 rad/s
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, ground,
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, 0),
                                 chrono.Q_from_AngAxis(0, 0, 1, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))
system.AddLink(motor)

# -------------------------------------------------------
# 4) Create the sun gear (rigidly attached to the end of the bar)
# -------------------------------------------------------
sun_rad = 0.2
sun = chrono.ChBody()
sun.SetMass(1.0)
sun.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
# place sun at x = +0.6 along the bar
sun.SetPos(chrono.ChVectorD(0.6, 0, 0))
# visual cylinder along Z, thickness 0.05
sun_cyl = chrono.ChCylinderShape()
sun_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(0.6, 0, -0.025)
sun_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(0.6, 0, +0.025)
sun_cyl.GetCylinderGeometry().rad = sun_rad
sun.AddAsset(sun_cyl)
sun.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.8, 0.2, 0.2)))
system.Add(sun)

# Lock sun gear to rotating bar (no relative motion)
lock_sun = chrono.ChLinkMateGeneric()
lock_sun.Initialize(sun, bar, chrono.ChFrameD())
# freeze all 6 DOFs
lock_sun.SetConstrainedCoords(True, True, True,  True, True, True)
system.AddLink(lock_sun)

# -------------------------------------------------------
# 5) Create the planet gear (pinned to ground)
# -------------------------------------------------------
planet_rad = 0.1
planet = chrono.ChBody()
planet.SetMass(0.5)
planet.SetInertiaXX(chrono.ChVectorD(0.005, 0.005, 0.005))
# place planet at x = +1.0 (to mesh with sun at 0.6 + 0.2)
planet.SetPos(chrono.ChVectorD(1.0, 0, 0))
# visual cylinder along Z, thickness 0.05
planet_cyl = chrono.ChCylinderShape()
planet_cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(1.0, 0, -0.025)
planet_cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(1.0, 0, +0.025)
planet_cyl.GetCylinderGeometry().rad = planet_rad
planet.AddAsset(planet_cyl)
planet.AddAsset(chrono.ChColorAsset(chrono.ChColor(0.2, 0.8, 0.2)))
system.Add(planet)

# Revolute joint planet <-> ground about Z through planet center
rev_planet = chrono.ChLinkLockRevolute()
rev_planet.Initialize(planet, ground,
                      chrono.ChFrameD(chrono.ChVectorD(1.0, 0, 0),
                                      chrono.Q_from_AngAxis(0, 0, 1, 0)))
system.AddLink(rev_planet)

# Gear constraint: omega_sun + (r_sun/r_planet)*omega_planet = 0  => ratio = -r_sun/r_planet
gear = chrono.ChLinkGear()
gear.Initialize(sun, planet,
                chrono.ChFrameD(chrono.ChVectorD(0,0,0),
                                chrono.Q_from_AngAxis(0,0,1,0)))
gear.SetGearRatio(-sun_rad/planet_rad)
system.AddLink(gear)

# -------------------------------------------------------
# 6) Set up Irrlicht visualization
# -------------------------------------------------------
app = chronoirr.ChIrrApp(system, "Epicyclic Gear Demo", chronoirr.dimension2du(1024,768))
app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddCamera(chronoirr.vector3df(1.5,1.0,1.5), chronoirr.vector3df(0.6,0,0))

# Must bind all assets before starting
app.AssetBindAll()
app.AssetUpdateAll()

# -------------------------------------------------------
# 7) Run the simulation
# -------------------------------------------------------
timestep = 1e-3
while app.Run():
    app.BeginScene()
    app.DrawAll()
    system.DoStepDynamics(timestep)
    app.EndScene()