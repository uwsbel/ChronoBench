import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath("../chrono_data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 1.0  
beam_height = 0.1  
beam_width = 0.05  
youngs_modulus = 2e11  
poissons_ratio = 0.3
density = 7850  


beam = fea.ChBeamSectionSteel(beam_width, beam_height)
beam.SetYoungModulus(youngs_modulus)
beam.SetPoissonRatio(poissons_ratio)
beam.SetDensity(density)


body = chrono.ChBodyEasy()
body.SetBodyFixed(False)
body.SetCollide(True)
body.SetMass(density * beam_width * beam_height * beam_length)  
body.AddAsset(beam)


link_revolute = chrono.ChLinkMateRevolute()
link_revolute.Initialize(body, None, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.AddLink(link_revolute)
body.SetPos(chrono.ChVectorD(0, 0.05, 0)) 


system.AddBody(body)


class AxialMotor(chrono.ChFunction):
    def __init__(self, body, force):
        chrono.ChFunction.__init__(self)
        self.body = body
        self.force = force

    def Get_Force(self):
        return chrono.ChVectorD(self.force, 0, 0)

    def Get_Force_X(self):
        return self.force

    def Get_Force_Y(self):
        return 0.0

    def Get_Force_Z(self):
        return 0.0


motor_force = 100.0  
motor = AxialMotor(body, motor_force)
body.AddForce(motor.Get_Force()) 



link_lock = chrono.ChLinkLock()
link_lock.Initialize(body, None, chrono.ChCoordsys(chrono.ChVectorD(beam_length, 0.05, 0)))
system.AddLink(link_lock)


system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)


timestepper = chrono.ChTimestepper.Type_EULER_IMPLICIT
system.SetTimestepperType(timestepper)
system.SetStep(0.001) 


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 1, -2))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.001)