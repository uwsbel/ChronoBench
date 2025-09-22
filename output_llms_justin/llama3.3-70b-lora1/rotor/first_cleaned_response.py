import pychrono as chrono
import pychrono.fea as fea
import pychrono.iga as iga
import pychrono.irrlicht as irr
import numpy as np
import math as m



beam = iga.ChIsogBeam(
    "beam",
    iga.ChIsogLineArcInit(
        chrono.ChVector3d(0, 0, 0),
        chrono.ChVector3d(0, 0, 1),
        0.1,
        20,
        True,
    ),
    iga.ChIsogLineSection(
        0.01,
        0.01,
        1287 * 1000,
        1000 * 1000,
        0.3,
        8100,
    ),
    iga.ChLineMaterialNSC(),
)


flywheel = iga.ChIsogMesh(
    "flywheel",
    iga.ChIsogCircleInit(
        chrono.ChVector3d(0, 0, 0.5),
        chrono.ChVector3d(0, 0, 1),
        0.1,
        20,
        0.1,
        20,
    ),
    iga.ChIsogCircleSection(
        0.01,
        8100,
        1000 * 1000,
        0.3,
    ),
    iga.ChCircleMaterial(),
)


sys = chrono.ChSystemNSC()


sys.Add(beam)
sys.Add(flywheel)


beam.SetInitialDisplacement(iga.ChIsogFrameInit(
    chrono.ChVector3d(0, 0, 0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)),
))


flywheel.SetInitialDisplacement(iga.ChIsogFrameInit(
    chrono.ChVector3d(0, 0, 0.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(1, 0, 0)),
))


flywheel.SetInitialAngularVelocity(chrono.ChVector3d(0, 0, 10))


motor = iga.ChIsogBeamMotor(beam, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))
sys.Add(motor)


joint = iga.ChIsogBeamRevolute(beam, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1))
sys.Add(joint)


mbs = chrono.ChCollisionSystemBullet()
sys.SetCollisionSystemType(mbs)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Jeffcott Rotor Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.0, 0.2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)