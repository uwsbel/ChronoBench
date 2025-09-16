import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


length = 1.0  
thickness = 0.01  
width = 0.1  
E = 2.1e9  
nu = 0.3  
rho = 7800  
force = 100  


sys = chrono.ChSystemNSC()


mesh = fea.ChMesh()
beam_L = fea.ChElementBeamEuler()
beam_L.SetNodes(fea.ChNodeFEAxyz(0, 0, 0), fea.ChNodeFEAxyz(length, 0, 0))
beam_L.SetSectionProperties(width, thickness)
beam_L.SetMaterialProperties(E, nu, rho)
mesh.AddElement(beam_L)


class CustomMotor(chrono.ChFunction):
    def __init__(self):
        super().__init__()
        self.t_ref = 0
        self.F = force

    def Get_y(self, x):
        if x > self.t_ref:
            self.t_ref = x
            return self.F
        else:
            return 0

motor = CustomMotor()


sys.AddConstraint(fea.ChLinkPointFrameGeneric(mesh.GetNode(0), None, False, False, False))
sys.AddConstraint(fea.ChLinkPointFrameGeneric(mesh.GetNode(1), None, False, False, False))


sys.Add(mesh)


solver = chrono.ChSolverBB()
timestepper = chrono.ChTimestepperHHT(solver)
sys.SetSolver(timestepper)


application = chronoirr.ChIrrApp(sys, 'PyChrono Beam Buckling Simulation', chronoirr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 1))
application.AddLightWithShadow(chronoirr.vector3df(2, 4, 2), chronoirr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512)


application.AssetBindAll()
application.AssetUpdateAll()
application.Simulate(10)