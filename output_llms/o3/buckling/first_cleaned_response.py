import math
import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.fea    as fea




chrono.SetChronoDataPath(chrono.GetChronoDataPath())   
sys = chrono.ChSystemSMC()                             


sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))           





beam_len   = 1.0                                       
beam_w     = 0.02                                      
E          = 210e9                                     
G          = 80e9                                      
rho        = 7800                                      




section = fea.ChBeamSectionAdvanced()
section.SetYoungModulus(E)
section.SetGwithPoissonRatio(E, 0.3)
section.SetDensity(rho)
section.SetAsRectangularSection(beam_w, beam_w, True)




mesh = fea.ChMesh()




n_elem = 20
dz     = beam_len / n_elem


nodes = []

for i in range(n_elem+1):
    node = fea.ChNodeFEAxyzrot( chrono.ChFrameD( chrono.ChVectorD( dz*i, 0, 0 ) ) )
    node.SetMass( rho * beam_w * beam_w * dz )         
    mesh.AddNode(node)
    nodes.append(node)

for i in range(n_elem):
    element = fea.ChElementBeamEuler()
    element.SetNodes( nodes[i], nodes[i+1] )
    element.SetSection( section )
    mesh.AddElement( element )

sys.Add(mesh)





ground = chrono.ChBody()
ground.SetBodyFixed(True)
sys.Add(ground)


ground_shape = chrono.ChBoxShape()
ground_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.05, 0.05, 0.05)
ground.AddAsset(ground_shape)


platen = chrono.ChBody()
platen.SetMass(1.0)
platen.SetPos( chrono.ChVectorD( beam_len, 0, 0 ) )
sys.Add(platen)


cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.02, -0.02, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD( 0.02,  0.02, 0)
cyl.GetCylinderGeometry().rad = 0.03
platen.AddAsset(cyl)



ground_constraint = fea.ChLinkNodeFrame()
ground_constraint.Initialize( nodes[0], ground )
sys.Add(ground_constraint)

platen_constraint = fea.ChLinkNodeFrame()
platen_constraint.Initialize( nodes[-1], platen )
sys.Add(platen_constraint)


prism = chrono.ChLinkLockPrismatic()
prism.Initialize( platen, ground,
                  chrono.ChCoordsysD( chrono.ChVectorD(beam_len, 0, 0),
                                      chrono.Q_from_AngAxis(math.pi/2, chrono.ChVectorD(0,0,1)) ) )
sys.Add(prism)


linmot = chrono.ChLinkMotorLinearPosition()
linmot.Initialize( platen, ground, chrono.ChFrameD( chrono.ChVectorD(0,0,0) ) )  




class CompressionFunction(chrono.ChFunction):
    def __init__(self, displacement=-0.02, ramp_time=0.5):
        chrono.ChFunction.__init__(self)
        self.disp = displacement
        self.tramp = ramp_time

    def Get_y(self, t):
        if t < self.tramp:
            return self.disp * (t/self.tramp)
        else:
            return self.disp

    def Get_y_dx(self, t):
        if t < self.tramp:
            return self.disp / self.tramp
        else:
            return 0.0

    def Get_y_dxdx(self, t):
        return 0.0

linmot.SetMotionFunction(CompressionFunction(displacement=-0.05, ramp_time=1.0))
sys.Add(linmot)





try:
    mkl = chrono.ChSolverMKL()
    mkl.SetVerbose(False)
    sys.SetSolver(mkl)
except:
    print("[INFO] MKL not available, switching to MINRES")
    minres = chrono.ChSolverMINRES()
    minres.SetMaxIterations(200)
    minres.SetTolerance(1e-10)
    minres.EnableDiagonalPreconditioner(True)
    sys.SetSolver(minres)


hht = chrono.ChTimestepperHHT()
hht.SetAlpha(-0.2)
hht.SetMaxiters(50)
hht.SetTolerance(1e-9)
sys.SetTimestepper(hht)




application = irr.ChIrrApp(sys, "Beam buckling / FEA demo", irr.dimension2du(1280,720))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalCamera(irr.vector3df(0.2, 0.3, 1.0))
application.AddTypicalLights()
application.AssetBindAll()
application.AssetUpdateAll()




step_size = 1e-3

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    

    sys.DoStepDynamics(step_size)

    application.EndScene()