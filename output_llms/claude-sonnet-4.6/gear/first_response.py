"""
PyChrono Epicyclic Gear System Simulation
==========================================
Demonstrates a planetary/epicyclic gear arrangement:
  - Fixed truss (ground)
  - Rotating bar (arm/carrier) driven by a gear motor
  - Sun gear (central, free to rotate relative to truss)
  - Planet gear (orbits around sun, meshed via gear constraint)
"""

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ============================================================
# 1. Create the simulation system
# ============================================================
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ============================================================
# 2. Epicyclic gear parameters
# ============================================================
msun_radius    = 0.30   # Sun gear radius [m]
mplanet_radius = 0.15   # Planet gear radius [m]
mthick         = 0.10   # Gear thickness [m]
marm_length    = msun_radius + mplanet_radius  # 0.45 m

# Gear ratio (planet/sun)
gear_tau = mplanet_radius / msun_radius  # 0.5

# Motor angular speed [rad/s]
motor_speed = math.pi  # 0.5 rev/s

# ============================================================
# 3. Create TRUSS – fixed ground body
# ============================================================
mbody_truss = chrono.ChBody()
mbody_truss.SetBodyFixed(True)
mbody_truss.SetName("Truss")
sys.Add(mbody_truss)

# Visual: a small box at the origin representing the frame
truss_shape = chrono.ChBoxShape()
truss_shape.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.10, 0.10, 0.35))
mat_truss = chrono.ChVisualMaterial()
mat_truss.SetDiffuseColor(chrono.ChColor(0.4, 0.4, 0.4))
mat_truss.SetSpecularColor(chrono.ChColor(0.6, 0.6, 0.6))
truss_shape.AddMaterial(mat_truss)
mbody_truss.AddVisualShape(truss_shape,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

# ============================================================
# 4. Create BAR – rotating carrier arm
# ============================================================
mbody_bar = chrono.ChBody()
mbody_bar.SetName("Bar")
mbody_bar.SetPos(chrono.ChVectorD(0, 0, 0))
mbody_bar.SetMass(1.0)
mbody_bar.SetInertiaXX(chrono.ChVectorD(0.01, 0.12, 0.12))
sys.Add(mbody_bar)

# Visual: a thin rectangular bar along X
bar_shape = chrono.ChBoxShape()
bar_shape.GetBoxGeometry().SetLengths(
    chrono.ChVectorD(marm_length + mplanet_radius + 0.02, 0.04, mthick * 0.6))
mat_bar = chrono.ChVisualMaterial()
mat_bar.SetDiffuseColor(chrono.ChColor(0.7, 0.7, 0.2))
mat_bar.SetSpecularColor(chrono.ChColor(0.9, 0.9, 0.5))
bar_shape.AddMaterial(mat_bar)
mbody_bar.AddVisualShape(bar_shape,
    chrono.ChFrameD(chrono.ChVectorD(marm_length / 2, 0, 0)))

# ============================================================
# 5. Create SUN GEAR – central gear body
# ============================================================
mbody_sun = chrono.ChBody()
mbody_sun.SetName("SunGear")
mbody_sun.SetPos(chrono.ChVectorD(0, 0, 0))
mbody_sun.SetMass(1.5)
mbody_sun.SetInertiaXX(
    chrono.ChVectorD(0.5 * 1.5 * msun_radius**2 * 0.5,
                     0.5 * 1.5 * msun_radius**2 * 0.5,
                     0.5 * 1.5 * msun_radius**2))
sys.Add(mbody_sun)

# Visual: cylinder aligned with Z axis
sun_cyl = chrono.ChCylinderShape()
sun_cyl.GetCylinderGeometry().rad = msun_radius
sun_cyl.GetCylinderGeometry().p1  = chrono.ChVectorD(0, 0, -mthick / 2)
sun_cyl.GetCylinderGeometry().p2  = chrono.ChVectorD(0, 0,  mthick / 2)
mat_sun = chrono.ChVisualMaterial()
mat_sun.SetDiffuseColor(chrono.ChColor(0.9, 0.5, 0.1))
mat_sun.SetSpecularColor(chrono.ChColor(1.0, 0.8, 0.4))
sun_cyl.AddMaterial(mat_sun)
mbody_sun.AddVisualShape(sun_cyl)

# Small axle cylinder for sun gear
sun_axle = chrono.ChCylinderShape()
sun_axle.GetCylinderGeometry().rad = 0.04
sun_axle.GetCylinderGeometry().p1  = chrono.ChVectorD(0, 0, -0.20)
sun_axle.GetCylinderGeometry().p2  = chrono.ChVectorD(0, 0,  0.20)
sun_axle.AddMaterial(mat_truss)
mbody_sun.AddVisualShape(sun_axle)

# ============================================================
# 6. Create PLANET GEAR – orbiting gear body
# ============================================================
mbody_planet = chrono.ChBody()
mbody_planet.SetName("PlanetGear")
mbody_planet.SetPos(chrono.ChVectorD(marm_length, 0, 0))
mbody_planet.SetMass(0.6)
mbody_planet.SetInertiaXX(
    chrono.ChVectorD(0.5 * 0.6 * mplanet_radius**2 * 0.5,
                     0.5 * 0.6 * mplanet_radius**2 * 0.5,
                     0.5 * 0.6 * mplanet_radius**2))
sys.Add(mbody_planet)

# Visual: cylinder aligned with Z axis
planet_cyl = chrono.ChCylinderShape()
planet_cyl.GetCylinderGeometry().rad = mplanet_radius
planet_cyl.GetCylinderGeometry().p1  = chrono.ChVectorD(0, 0, -mthick / 2)
planet_cyl.GetCylinderGeometry().p2  = chrono.ChVectorD(0, 0,  mthick / 2)
mat_planet = chrono.ChVisualMaterial()
mat_planet.SetDiffuseColor(chrono.ChColor(0.2, 0.6, 0.9))
mat_planet.SetSpecularColor(chrono.ChColor(0.5, 0.8, 1.0))
planet_cyl.AddMaterial(mat_planet)
mbody_planet.AddVisualShape(planet_cyl)

# Small axle cylinder for planet
planet_axle = chrono.ChCylinderShape()
planet_axle.GetCylinderGeometry().rad = 0.025
planet_axle.GetCylinderGeometry().p1  = chrono.ChVectorD(0, 0, -0.15)
planet_axle.GetCylinderGeometry().p2  = chrono.ChVectorD(0, 0,  0.15)
planet_axle.AddMaterial(mat_truss)
mbody_planet.AddVisualShape(planet_axle)

# ============================================================
# 7. Constraints and motor
# ============================================================

# --- Motor: rotates the bar (carrier) at constant speed relative to truss ---
# The ChLinkMotorRotationSpeed drives rotation about the joint's Z axis.
# With identity frame orientation (QUNIT) the rotation is about world Z.
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(
    mbody_bar, mbody_truss,
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
my_motor.SetSpeedFunction(chrono.ChFunction_Const(motor_speed))
sys.Add(
print("error happened with only start ```python")