import pychrono as chrono
import numpy as np


chrono.setChronoDataPath("path/to/chrono_data")  
chrono.setChronoLogLevel(chrono.LOG_LEVEL_WARNING)  


simulation = chrono.ChSystemSMC()
solver = chrono.ChSolverSD()
solver.SetMaxIters(50)
solver.SetTol(1e-3)
simulation.SetSolver(solver)


timestepper = chrono.ChTimestepperHHT()
simulation.SetTimestepper(timestepper)


vis = chrono.ChVisualSystemIrrlicht()
vis.Attach(simulation)
vis.Initialize()
vis.SetWindowTitle("Beam Buckling Simulation")
vis.SetWindowSize(800, 600)


beam = chrono.ChBeam()
beam.SetMaterial(chrono.MaterialSurfaceNSC())
beam.SetSection(chrono.ChBeamSectionEuler(1.0, 0.1))  
beam.SetLength(1.0)  
beam.SetNumElements(10)  
beam.SetFixed(0)  


class BeamMotor(chrono.ChFunction):
    def __call__(self, t):
        if t < 1.0:
            return 10.0 * t  
        else:
            return 10.0  

motor = BeamMotor()
force = chrono.ChForce()
force.SetForce(chrono.VECT_Z, motor)
beam.GetNode(beam.GetNNodes() - 1).AddForce(force)  


simulation.Add(beam)


constraint = chrono.ChConstraintSpherical()
constraint.Initialize(beam.GetNode(0), chrono.ChVectorD(0, 0, 0))
simulation.Add(constraint)


while vis.Run():
    simulation.DoStepDynamics(0.01)  
    vis.Render()  

vis.Shutdown()