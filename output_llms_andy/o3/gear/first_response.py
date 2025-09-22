####################################################################
#  Epicyclic gear-set demo – Project Chrono (Python)
#  ---------------------------------------------------------------
#  • fixed truss (ground)
#  • carrier / bar that rotates about the Z axis
#  • two cylindrical gears (sun & planet)
#  • one speed motor that drives the sun gear
#  • Irrlicht visualization with sky, light, camera, materials
####################################################################

import pychrono as chrono
import pychrono.irrlicht as irr
import math

# ---------------------------------------------------------------
# 1. Chrono initialisation
# ---------------------------------------------------------------
chrono.SetChronoDataPath( chrono.GetChronoDataPath() )   # textures etc.

sys = chrono.ChSystemNSC()
sys.Set_G_acc( chrono.ChVectorD(0, 0, -9.81) )

# Some reusable contact material
mat = chrono.ChMaterialSurfaceNSC()
mat.SetFriction(0.3)

# ---------------------------------------------------------------
# 2. Bodies, joints, and motor
# ---------------------------------------------------------------

# ---- 2.1  Fixed truss (ground) ---------------------------------
truss = chrono.ChBodyEasyBox(0.20, 0.20, 0.20,      # size
                             1000,                  # density
                             True, True, mat)       # visual, collide, material
truss.SetBodyFixed(True)
truss.SetPos( chrono.ChVectorD(0,0,0) )
sys.Add(truss)

# ---- 2.2  Rotating bar / carrier -------------------------------
bar_len   = 0.60
bar_width = 0.05
bar = chrono.ChBodyEasyBox(bar_len, bar_width, bar_width,
                           800, True, True, mat)
# its COM at half-length so the left end is at the origin
bar.SetPos( chrono.ChVectorD(bar_len*0.5, 0, 0) )
sys.Add(bar)

# Revolute joint (Z axis) between ground and carrier at origin
rev_ground_bar = chrono.ChLinkLockRevolute()
rev_ground_bar.Initialize(truss, bar,
        chrono.ChCoordsysD( chrono.ChVectorD(0,0,0),
                            chrono.Q_from_AngAxis(0, chrono.ChVectorD(0,1,0)) ))
sys.Add(rev_ground_bar)

# ---- 2.3  Sun gear (gear 1) ------------------------------------
r_sun   = 0.10
thick   = 0.05
sun = chrono.ChBodyEasyCylinder(r_sun, thick,
                                7800, True, True, mat)
sun.SetPos( chrono.ChVectorD(0,0,0) )
sys.Add(sun)

# Speed-controlled motor between sun gear and ground
motor_sun = chrono.ChLinkMotorRotationSpeed()
motor_sun.Initialize(sun, truss,
        chrono.ChFrameD( chrono.ChVectorD(0,0,0), chrono.QUNIT ) )

const_speed = chrono.ChFunction_Const( 2*math.pi )   # 1 rev/s  (≈ 60 rpm)
motor_sun.SetSpeedFunction(const_speed)
sys.Add(motor_sun)

# ---- 2.4  Planet gear (gear 2) ---------------------------------
r_planet = 0.20
planet   = chrono.ChBodyEasyCylinder(r_planet, thick,
                                     7800, True, True, mat)
# place it at the right end of the carrier
planet_center = chrono.ChVectorD( r_sun + r_planet, 0, 0 )   # 0.30 m
planet.SetPos( planet_center )
sys.Add(planet)

# Revolute joint between carrier and planet (planet spins about its Z axis,
# but its centre is locked to the bar’s end point)
rev_bar_planet = chrono.ChLinkLockRevolute()
rev_bar_planet.Initialize(bar, planet,
        chrono.ChCoordsysD( planet_center, chrono.QUNIT ))
sys.Add(rev_bar_planet)

# ---- 2.5  Ideal (kinematic) gear constraint --------------------
#   External mesh: ratio = - R_planet / R_sun
ratio = - r_sun / r_planet
gear_link = chrono.ChLinkGear()
# Local reference frame for the constraint (located at the sun axis, Z up)
gear_frame = chrono.ChFrameD( chrono.ChVectorD(0,0,0), chrono.QUNIT )
gear_link.Initialize(sun, planet,
                     gear_frame,                   # frame on body 1
                     chrono.ChVectorD(0,0,1),      # common rotation axis
                     chrono.ChVectorD(1,0,0),      # direction to planet
                     ratio,                        # transmission ratio
                     False)                        # external gears
sys.Add(gear_link)

# ---------------------------------------------------------------
# 3. Irrlicht visualisation
# ---------------------------------------------------------------
app = irr.ChIrrApp(sys, "Epicyclic Gear Set", irr.dimension2du(1024,768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera( irr.vector3df(0.6,0.6,0.6), irr.vector3df(0,0,0) )
app.AddTypicalLights( irr.vector3df( 3, 3, 3 ),
                      irr.vector3df(-3,-3, 5 ), 120.0, 1.0 )

# Make assets visible
app.AssetBindAll()
app.AssetUpdateAll()

# ---------------------------------------------------------------
# 4. Simulation loop
# ---------------------------------------------------------------
app.SetTimestep(1e-3)

while(app.GetDevice().run()):
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()