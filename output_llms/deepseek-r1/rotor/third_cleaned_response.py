import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemSMC()

mesh = fea.ChMesh()
system.Add(mesh)

mesh.SetAutomaticGravity(True, 2)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

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


mbodyflywheel = chrono.ChBodyEasyCylinder(0.24, 0.1, 7800, chrono.ChAxis_Y)
mbodyflywheel.SetCoordsys(
    chrono.ChCoordsysd(node_mid.GetPos() + chrono.ChVector3d(0, 0.05, 0),
                       chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Z))
)
system.Add(mbodyflywheel)


myjoint = chrono.ChLinkMateFix()
myjoint.Initialize(node_mid, mbodyflywheel)
system.Add(myjoint)


truss = chrono.ChBody()
truss.SetFixed(True)
system.Add(truss)

bearing = chrono.ChLinkMateGeneric(False, True, True, False, True, True)
bearing.Initialize(builder.GetLastBeamNodes().back(), truss,
                   chrono.ChFramed(builder.GetLastBeamNodes().back().GetPos()))
system.Add(bearing)


class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.A1 = 40.0
        self.A2 = 20.0
        self.T1 = 0.5
        self.T2 = 1.0
        self.T3 = 1.5
        self.w = 0.2

    def GetVal(self, x):
        if x < self.T1:
            return self.A1 * m.sin(self.w * x)
        elif x < self.T2:
            return self.A2 * m.sin(self.w * x)
        elif x < self.T3:
            return self.A1 * m.cos(self.w * x)
        else:
            return self.A2 * m.cos(self.w * x)


rotmotor1 = chrono.ChLinkMotorRotationSpeed()
rotmotor1.Initialize(builder.GetLastBeamNodes().front(), truss,
                    chrono.ChFramed(builder.GetLastBeamNodes().front().GetPos(),
                                    chrono.QuatFromAngleAxis(m.pi / 2.0, chrono.VECT_Y)))
system.Add(rotmotor1)
rotmotor1.SetMotorFunction(ChFunctionMyFun())


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
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Jeffcott Rotor with Custom Motor Function')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1, 4), chrono.ChVector3d(beam_L / 2, 0, 0))
vis.AddTypicalLights()


msolver = mkl.ChSolverPardisoMKL()
system.SetSolver(msolver)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.Initialize()

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.002)