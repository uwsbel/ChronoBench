import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

# =============================================================================
# Initialise the Chrono simulation system
# =============================================================================
sys = chrono.ChSystemNSC()

# ----- Gravity (Moon) -----
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -1.62, 0))

# =============================================================================
# Ground body
# =============================================================================
ground = chrono.ChBody()
sys.AddBody(ground)               # corrected method for adding a body
ground.SetFixed(True)
ground.EnableCollision(False)

# Visual cylinder for the ground (unchanged)
cyl_ground = chrono.ChVisualShapeCylinder(0.2, 0.4)
ground.AddVisualShape(cyl_ground, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))

# =============================================================================
# Pendulum body
# =============================================================================
pend_1 = chrono.ChBody()
sys.AddBody(pend_1)
pend_1.SetFixed(False)
pend_1.EnableCollision(False)

# ----- Mass & inertia (modified) -----
pend_1.SetMass(2)                                 # 2 kg
pend_1.SetInertiaXX(chrono.ChVector3d(0.4, 1.5, 1.5))  # kg·m²

# ----- Visual cylinder (modified dimensions) -----
cyl_pend = chrono.ChVisualShapeCylinder(0.1, 1.5)   # radius 0.1 m, height 1.5 m
cyl_pend.SetColor(chrono.ChColor(0.6, 0, 0))       # red colour
pend_1.AddVisualShape(cyl_pend,
                      chrono.ChFramed(chrono.VNULL,
                                      chrono.QuatFromAngleY(chrono.CH_PI_2)))

# ----- Initial position (centre of mass) -----
pend_1.SetPos(chrono.ChVector3d(1, 0, 1))

# ----- Initial angular velocity (added) -----
pend_1.SetAngVel(chrono.ChVector3d(0, 0, 2))       # 2 rad/s about Z

# =============================================================================
# Spherical joint (replaces the original revolute joint)
# =============================================================================
spherical_link = chrono.ChLinkLockSpherical()
spherical_link.Initialize(ground, pend_1,
                          chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                          chrono.ChQuaterniond(1, 0, 0, 0)))
sys.AddLink(spherical_link)

# ----- Joint visualisation: sphere with radius 2 -----
sphere_vis = chrono.ChVisualShapeSphere(2)
sphere_vis.SetColor(chrono.ChColor(0, 1, 0))   # green colour
spherical_link.AddVisualShape(sphere_vis,
                              chrono.ChFramed(chrono.ChVector3d(0, 0, 1),
                                              chrono.ChQuaterniond(1, 0, 0, 0)))

# =============================================================================
# Irrlicht visualisation
# =============================================================================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Spherical joint pendulum on Moon')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# =============================================================================
# Simulation loop
# =============================================================================
log_info = True
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(1e-3)            # 1 ms time step

    # ----- Log position & velocity after 1 s of simulation time -----
    if log_info and sys.GetChTime() > 1:
        pos_1 = pend_1.GetPos()
        print("t =", sys.GetChTime())
        print("    Position:", pos_1.x, pos_1.y, pos_1.z)
        lin_vel_1 = pend_1.GetPosDt()
        print("    Linear velocity:", lin_vel_1.x, lin_vel_1.y, lin_vel_1.z)
        log_info = False