import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.motion as chmotion

chrono.SetChronoDataPath(chrono.GetChronoDataPath())
print(chrono.GetChronoDataPath())

mphysicalSystem = chrono.ChSystemNSC()
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


mfloor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True)
mfloor.SetPos(chrono.ChVector3d(0, 0, -1))
mfloor.SetFixed(True)
mfloor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
mphysicalSystem.Add(mfloor)


mcrankshaft = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 2, 1, 1000, True, True)
mcrankshaft.SetPos(chrono.ChVector3d(0, 3, 0))
mcrankshaft.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI_2, chrono.ChVector3d(1, 0, 0)))
mcrankshaft.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
mphysicalSystem.Add(mcrankshaft)


mrod = chrono.ChBodyEasyBox(0.1, 0.1, 3, 1000, True, True)
mrod.SetPos(chrono.ChVector3d(0, 0, 1))
mrod.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/green.png"))
mphysicalSystem.Add(mrod)


mpiston = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.4, 1, 1000, True, True)
mpiston.SetPos(chrono.ChVector3d(0, -3, 0))
mpiston.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/red.png"))
mphysicalSystem.Add(mpiston)


my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrankshaft, mfloor, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))
my_motor.SetSpeedFunction(chrono.ChFunctionConst(chrono.CH_PI_2))  
mphysicalSystem.Add(my_motor)


mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod, mcrankshaft, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))
mphysicalSystem.Add(mjointA)


mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston, mrod, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))
mphysicalSystem.Add(mjointB)


mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston, mfloor, chrono.ChFramed(chrono.ChVector3d(0, 0, 1), chrono.QUNIT))
mphysicalSystem.Add(mjointC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mphysicalSystem)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 6))
vis.AddTypicalLights()


mtrajectory = chronoirr.ChVisualSystemIrrlicht()
mtrajectory.AttachSystem(mphysicalSystem)
mtrajectory.SetWindowSize(400, 400)
mtrajectory.SetWindowTitle('motion visualization')
mtrajectory.Initialize()
mtrajectory.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
mtrajectory.AddSkyBox()
mtrajectory.AddCamera(chrono.ChVector3d(0, 0, 6))
mtrajectory.AddTypicalLights()


vis.SetDebuggerStrokeStyle(chrono.ChVisualSystem.DEBUGGER_STROKE_STYLE_LINE)


class MyCallback(chrono.ChIrrEventReceiver):
    def __init__(self):
        self.my_text = ''

    def OnGetLog(self, log):
        self.my_text = log

vis.RegisterEventReceiver(MyCallback())


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    mphysicalSystem.DoStepDynamics(1e-3)
    if not mtrajectory.GetIsInitialized():
        mtrajectory.Initialize()
    mtrajectory.DrawAll()