import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

sys = chrono.ChSystemSMC()

mesh = fea.ChMesh()
sys.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

beam_L = 6
beam_ro = 0.050
beam_ri = 0.045


minertia = fea.ChInertiaCosseratSimple()
minertia.SetDensity(7800)
minertia.SetArea(m.pi * (pow(beam_ro, 2) - pow(beam_ri, 2)))
minertia.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
minertia.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(210e9)
melasticity.SetShearModulusFromPoisson(0.3)
melasticity.SetIyy((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetIzz((m.pi / 4.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))
melasticity.SetJ((m.pi / 2.0) * (pow(beam_ro, 4) - pow(beam_ri, 4)))

msection = fea.ChBeamSectionCosserat(minertia, melasticity)
msection.SetCircular(True)
msection.SetDrawCircularRadius(beam_ro)


builder = fea.ChBuilderBeamIGA()
builder.BuildBeam(mesh, msection, 20,
                  chrono.ChVector3d(0, 0, 0),
                  chrono.ChVector3d(beam_L, 0, 0),
                  chrono.VECT_Y, 1)

node_mid = builder.GetLastBeamNodes()[len(builder.GetLastBeamNodes()) // 2]


mbodyflywheel = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, 0.24, 0.1, 7800)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Z))
)
sys.Add(mbodyflywheel)

myjoint = fea.ChLinkNodeBody()
myjoint.Initialize(node_mid, mbodyflywheel)
sys.Add(myjoint)


truss = chrono.ChBody()
truss.SetFixed(True)
sys.Add(truss)


last_node = builder.GetLastBeamNodes().back()
bearing_dummy = chrono.ChBody()
bearing_dummy.SetPos(last_node.GetPos())
sys.Add(bearing_dummy)

link_bearing = fea.ChLinkNodeBody()
link_bearing.Initialize(last_node, bearing_dummy)
sys.Add(link_bearing)

bearing = chrono.ChLinkLockRevolute()
bearing.Initialize(bearing_dummy, truss, chrono.ChFramed(last_node.GetPos(), chrono.QUNIT))
sys.Add(bearing)


first_node = builder.GetLastBeamNodes().front()
motor_dummy = chrono.ChBody()
motor_dummy.SetPos(first_node.GetPos())
sys.Add(motor_dummy)

link_motor = fea.ChLinkNodeBody()
link_motor.Initialize(first_node, motor_dummy)
sys.Add(link_motor)

class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.A1 = 40.0
        self.A2 = 0.2
        self.T1 = 1.0
        self.T2 = 2.0
        self.T3 = 3.0
        self.w = 2.0

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif x < self.T2:
            return self.A2 * x
        else:
            return self.A2 * self.T2

rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(motor_dummy, truss,
                    chrono.ChFramed(first_node.GetPos(),
                                    chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Y)))
rotmotor1.SetMotorFunction(ChFunctionMyFun())
sys.Add(rotmotor1)


mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizebeamA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Test FEA: Jeffcott rotor with IGA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)







while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)