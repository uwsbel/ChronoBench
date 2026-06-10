import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
#  1. Initialize the PyChrono environment and core components
# ---------------------------------------------------------------------
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  # gears in a plane, gravity off

# A contact material (shared by the bodies)
mat = chrono.ChContactMaterialNSC()

# ---------------------------------------------------------------------
#  Geometry parameters
# ---------------------------------------------------------------------
radA = 1.5          # radius of the sun (fixed/central) gear
wheel_thickness = 0.5
radB = 1.0          # radius of the planet gear
interaxis = radA + radB   # distance between the two gear centers


# ---------------------------------------------------------------------
#  2. Add the required physical systems and objects
# ---------------------------------------------------------------------

# --- Fixed truss (the static frame) ---------------------------------
truss = chrono.ChBodyEasyBox(interaxis + 4.2, 1.5, 0.5,   # X Y Z size
                             1000,    # density
                             True,    # visualize
                             False,   # no collision
                             mat)
truss.SetPos(chrono.ChVector3d(0, 0, 0))
truss.SetFixed(True)
sys.Add(truss)

# --- Central (sun) gear : FIXED to the truss ------------------------
gearA = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                  radA,            # radius
                                  wheel_thickness, # height
                                  1000,            # density
                                  True, False, mat)
gearA.SetPos(chrono.ChVector3d(0, 0, 0))
# rotate cylinder so the wheel axis lies along Z
gearA.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
gearA.GetVisualShape(0).SetColor(chrono.ChColor(1.0, 0.4, 0.4))
sys.Add(gearA)

# Lock the sun gear to the truss (it does not turn)
link_truss_gearA = chrono.ChLinkLockLock()
link_truss_gearA.Initialize(gearA, truss,
                            chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(link_truss_gearA)

# --- Rotating bar (planet carrier / arm) ----------------------------
bar = chrono.ChBodyEasyBox(interaxis + 0.4, 0.4, 0.4,
                           1000, True, False, mat)
# Place its center half-way between the two gear axes
bar.SetPos(chrono.ChVector3d(interaxis / 2.0, 0, -1.0))
bar.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.4, 0.9))
sys.Add(bar)

# Revolute joint: bar rotates about the central (sun) gear axis (Z-axis)
link_bar = chrono.ChLinkLockRevolute()
link_bar.Initialize(bar, truss,
                    chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
sys.Add(link_bar)

# --- Planet gear (orbits around the sun gear) -----------------------
gearB = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y,
                                  radB,
                                  wheel_thickness,
                                  1000, True, False, mat)
gearB.SetPos(chrono.ChVector3d(interaxis, 0, 0))
gearB.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
gearB.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.9, 0.4))
sys.Add(gearB)

# Revolute joint connecting planet gear to the tip of the rotating bar
link_planet = chrono.ChLinkLockRevolute()
link_planet.Initialize(gearB, bar,
                        chrono.ChFramed(chrono.ChVector3d(interaxis, 0, 0)))
sys.Add(link_planet)


# ---------------------------------------------------------------------
#  3. Set up the gear constraints and the gear motor
# ---------------------------------------------------------------------

# --- Gear constraint between sun gear (A) and planet gear (B) -------
# This enforces the proper kinematic gear ratio (rolling without slip).
gear_constraint = chrono.ChLinkLockGear()
gear_constraint.Initialize(gearA, gearB,
                           chrono.ChFramed())
gear_constraint.SetFrameShaft1(
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                    chrono.QuatFromAngleX(-chrono.CH_PI_2)))
gear_constraint.SetFrameShaft2(
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                    chrono.QuatFromAngleX(-chrono.CH_PI_2)))
gear_constraint.SetTransmissionRatio(radA / radB)
gear_constraint.SetEpicyclic(True)   # epicyclic (planetary) behaviour
sys.AddLink(gear_constraint)

# --- Gear motor: drive the rotating bar at constant speed -----------
# The motor rotates the carrier (bar) relative to the truss.
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, truss,
                 chrono.ChFramed(chrono.ChVector3d(0, 0, 0),
                                 chrono.QuatFromAngleX(chrono.CH_PI_2)))
motor.SetSpeedFunction(chrono.ChFunctionConst(0.5))   # constant 0.5 rad/s
sys.AddLink(motor)


# ---------------------------------------------------------------------
#  4. Visualization with Irrlicht
# ---------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1000, 800)
vis.SetWindowTitle('Epicyclic Gears Demo')
vis.Initialize()
vis.AddLogo(chronoirr.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 4, -8),
              chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(5, 8, -5),
                       chrono.ChVector3d(0, 0, 0),
                       12, 2, 12, 55)


# ---------------------------------------------------------------------
#  5. Simulation loop
# ---------------------------------------------------------------------
sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)

while vis.Run():
    vis.BeginScene()
    vis.Render()
    # draw the two gear axes for clarity
    chronoirr.drawSegment(vis,
                          chrono.ChVector3d(0, 0, 0),
                          chrono.ChVector3d(interaxis, 0, 0),
                          chrono.ChColor(0, 1, 0), True)
    vis.EndScene()

    sys.DoStepDynamics(1e-3)