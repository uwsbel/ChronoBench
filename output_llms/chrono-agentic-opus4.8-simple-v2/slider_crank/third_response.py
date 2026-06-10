import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                            # NSC rigid-body system
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))      # g = 9.81 along -Y (Y-up)

crank_rad = 1.0       # crank radius [m] — pin offset from the crank axis
rod_len = 4.0         # connecting-rod length [m]
crank_z = 0.0         # mechanism lies in the z = 0 (x-y) plane

# floor / truss — the fixed reference body
floor = chrono.ChBodyEasyBox(10, 6, 1, 1000, True, False)            # visualize only, no collision
floor.SetPos(chrono.ChVector3d(0, 0, 2.5))                           # behind the mechanism plane (away from camera)
floor.SetFixed(True)                                                 # truss is fixed to ground
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
sys.Add(floor)

# crank — a flat plate rotating about the world Z axis at the origin
crank = chrono.ChBody()                                              # manual body for full control
crank.SetMass(2.0)                                                   # crank mass [kg]
crank.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))                 # crank inertia tensor (diag)
crank.SetPos(chrono.ChVector3d(crank_rad / 2.0, 0, crank_z))        # centered between axis and pin
sys.Add(crank)
crank_box = chrono.ChVisualShapeBox(crank_rad + 0.4, 0.4, 0.2)      # plate spanning axis to pin
crank_box.SetColor(chrono.ChColor(0.6, 0.2, 0.2))                   # red-ish crank
crank.AddVisualShape(crank_box)
crank.AddVisualShape(chrono.ChVisualShapeCylinder(0.1, 0.4),        # axis stub at the rotation centre
    chrono.ChFramed(chrono.ChVector3d(-crank_rad / 2.0, 0, 0), chrono.QUNIT))
crank.AddVisualShape(chrono.ChVisualShapeCylinder(0.1, 0.4),        # crank-pin stub at the rod end
    chrono.ChFramed(chrono.ChVector3d(crank_rad / 2.0, 0, 0), chrono.QUNIT))

# connecting rod — slim cylinder from the crank pin to the piston
rod = chrono.ChBody()                                               # manual body (orientation changes)
rod.SetMass(1.0)                                                    # rod mass [kg]
rod.SetInertiaXX(chrono.ChVector3d(0.01, 0.5, 0.5))                 # slender-rod inertia
rod.SetPos(chrono.ChVector3d(crank_rad + rod_len / 2.0, 0, crank_z))  # centered crank-pin → piston
sys.Add(rod)
rod_cyl = chrono.ChVisualShapeCylinder(0.07, rod_len)              # thin rod visual, default Z axis
rod_cyl.SetColor(chrono.ChColor(0.2, 0.2, 0.6))                    # blue-ish rod
rod.AddVisualShape(rod_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # Z → X

# piston — a block sliding along the +X guide
piston = chrono.ChBody()                                           # manual body for the slider
piston.SetMass(1.5)                                                # piston mass [kg]
piston.SetInertiaXX(chrono.ChVector3d(0.2, 0.2, 0.2))             # piston inertia tensor
piston.SetPos(chrono.ChVector3d(crank_rad + rod_len, 0, crank_z)) # at the far end of the rod
sys.Add(piston)
piston_cyl = chrono.ChVisualShapeCylinder(0.5, 0.8)              # cylindrical piston visual
piston_cyl.SetColor(chrono.ChColor(0.2, 0.5, 0.2))              # green-ish piston
piston.AddVisualShape(piston_cyl, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))  # axis → X

# crank ↔ floor : prescribed-speed motor (a FULL motor-link — no companion revolute)
motor = chrono.ChLinkMotorRotationSpeed()                         # spins the crank
motor.Initialize(crank, floor, chrono.ChFramed(chrono.ChVector3d(0, 0, crank_z)))  # axis at the origin, about Z
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))     # constant angular speed π rad/s
sys.AddLink(motor)

# crank ↔ rod : SPHERICAL (ball-and-socket) at the crank pin — body-local frames
joint_crank_rod = chrono.ChLinkLockSpherical()
joint_crank_rod.Initialize(crank, rod, True,
    chrono.ChFramed(chrono.ChVector3d(crank_rad / 2.0, 0, 0)),    # crank-pin point on the crank
    chrono.ChFramed(chrono.ChVector3d(-rod_len / 2.0, 0, 0)))     # near end of the rod
sys.AddLink(joint_crank_rod)

# rod ↔ piston : SPHERICAL (ball-and-socket) at the wrist pin — body-local frames
joint_rod_piston = chrono.ChLinkLockSpherical()
joint_rod_piston.Initialize(rod, piston, True,
    chrono.ChFramed(chrono.ChVector3d(rod_len / 2.0, 0, 0)),      # far end of the rod
    chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))                  # piston centre
sys.AddLink(joint_rod_piston)

# piston ↔ floor : PLANAR (plane-plane) — constrains the piston to move and rotate in the x-y plane.
# ChLinkLockPlanar removes z-translation + x/y-rotation; its frame +Z is the plane normal (world +Z).
joint_piston_floor = chrono.ChLinkLockPlanar()
joint_piston_floor.Initialize(piston, floor,
    chrono.ChFramed(chrono.ChVector3d(crank_rad + rod_len, 0, crank_z), chrono.QUNIT))  # normal = +Z
sys.AddLink(joint_piston_floor)

vis = chronoirr.ChVisualSystemIrrlicht()                          # Irrlicht render window
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)                                     # window resolution
vis.SetWindowTitle("Slider-Crank (spherical pins, planar piston)")
vis.Initialize()                                                # device first, then scene nodes
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 4, -8), chrono.ChVector3d(2.5, 0, 0))  # view the mechanism plane
vis.AddTypicalLights()

time_step = 1e-3       # integration step [s]
sim_end = 10.0         # total simulated time [s]
render_fps = 50.0      # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged render-cadence constant
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
