import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemNSC()                                          # NSC rigid-body system, default Y-down gravity (0, -9.81, 0)

crank_center = chrono.ChVector3d(-1, 0.5, 0)                        # crank rotation center
crank_rad = 0.4                                                     # crank radius
crank_thick = 0.1                                                  # crank disc thickness
rod_length = 1.5                                                   # connecting-rod length

floor = chrono.ChBodyEasyBox(3, 1, 3, 1000)                        # truss/floor box, density 1000
floor.SetPos(chrono.ChVector3d(0, -0.5, 0))                        # floor sits below origin
floor.SetFixed(True)                                               # floor is the fixed truss
floor.SetName("floor")
floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # floor texture
sys.Add(floor)

crank = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, crank_rad, crank_thick, 1000)   # crank disc
crank.SetPos(crank_center + chrono.ChVector3d(0, 0, -0.1))         # offset crank in -Z
crank.SetRot(chrono.Q_ROTATE_Y_TO_Z)                               # lay disc axis along Z
crank.SetName("crank")
crank.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))  # crank texture
sys.Add(crank)

rod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)             # connecting rod box
rod.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length / 2, 0, 0))     # rod between crank-pin and wrist-pin
rod.SetName("rod")
rod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/pinkwhite.png"))    # rod texture
sys.Add(rod)

piston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.2, 0.3, 1000)                # piston cylinder
piston.SetPos(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0))      # piston at far end of rod
piston.SetRot(chrono.Q_ROTATE_Y_TO_X)                              # lay piston axis along X (slide direction)
piston.SetName("piston")
piston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))  # piston texture
sys.Add(piston)

motor = chrono.ChLinkMotorRotationSpeed()                          # speed motor = full motor-link (no extra revolute)
motor.Initialize(crank, floor, chrono.ChFramed(crank_center))      # drive crank vs floor at crank center
motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI))       # constant π rad/s
sys.AddLink(motor)

revoluteA = chrono.ChLinkLockRevolute()                            # crank-pin: rod <-> crank
revoluteA.Initialize(rod, crank, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad, 0, 0)))
sys.AddLink(revoluteA)

revoluteB = chrono.ChLinkLockRevolute()                            # wrist-pin: piston <-> rod
revoluteB.Initialize(piston, rod, chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0)))
sys.AddLink(revoluteB)

prismatic = chrono.ChLinkLockPrismatic()                           # piston slides on the fixed guide
prismatic.Initialize(piston, floor,
                     chrono.ChFramed(crank_center + chrono.ChVector3d(crank_rad + rod_length, 0, 0),
                                     chrono.Q_ROTATE_Z_TO_X))       # guide axis along X
sys.AddLink(prismatic)

vis = chronoirr.ChVisualSystemIrrlicht()                           # Irrlicht window
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)                                       # window size
vis.SetWindowTitle("Crank-Slider Mechanism")                       # window title
vis.Initialize()                                                   # create device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # logo (after Initialize)
vis.AddSkyBox()                                                    # sky box
vis.AddCamera(chrono.ChVector3d(1, 1, 3), chrono.ChVector3d(0, 1, 0))   # camera eye, target
vis.AddTypicalLights()                                             # standard lights

time_step = 1e-3                                                   # integration step
render_fps = 50.0                                                  # review video fps
render_every = max(1, round(1.0 / (render_fps * time_step)))       # untagged cadence constant
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        sys.DoStepDynamics(time_step)
