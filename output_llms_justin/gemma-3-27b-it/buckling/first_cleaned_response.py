import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea


chrono.SetChronoDataPath("../chrono_data/")  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


beam_length = 1.0  
beam_height = 0.1  
beam_width = 0.05  
beam_density = 7850  
youngs_modulus = 2e11  
poissons_ratio = 0.3


beam = fea.ChBeamFiniteElement(beam_length, beam_height, beam_width, beam_density, youngs_modulus, poissons_ratio)
beam.MapMaterialProperties(youngs_modulus, poissons_ratio)
beam.SetYoungModulus(youngs_modulus)
beam.SetPoissonRatio(poissons_ratio)


system.Add(beam)


beam.Set_Fix_X(True)
beam.Set_Fix_Y(True)
beam.Set_Fix_Z(True)
beam.Set_Fix_WX(True)
beam.Set_Fix_WY(False)  
beam.Set_Fix_WZ(True)


force_magnitude = 100  
force_point = chrono.ChVectorD(beam_length, 0, 0)
system.Add_Force(force_point, chrono.ChVectorD(0, -force_magnitude, 0))



class CustomMotor:
    def __init__(self, beam, force_point, magnitude):
        self.beam = beam
        self.force_point = force_point
        self.magnitude = magnitude
        self.time = 0.0

    def ApplyForce(self, time):
        self.time = time
        force_value = self.magnitude * chrono.ChMath.sin(2 * chrono.CH_C_PI * time)  
        force = chrono.ChVectorD(0, -force_value, 0)
        self.beam.ApplyForce(self.force_point, force)







system.SetSolverType(chrono.ChSolver.Type_SOR)
system.SetSolverMaxIterations(100)
system.SetSolverTolerance(1e-6)
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT)
system.SetStepTime(0.001)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Beam Buckling Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(beam_length * 1.5, beam_height * 2, -beam_length * 1.5))
vis.AddTypicalLights()


time_end = 5.0
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(0.001)

    
    

    if system.GetChTime() > time_end:
        break

vis.GetIrrlichtApplication().Close()