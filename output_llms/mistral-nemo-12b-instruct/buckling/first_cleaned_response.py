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
beam.SetNumberOfElements(10)  
beam.SetInitialPosition(chrono.ChVectorD(0, 0, 0))  
beam.SetInitialRotation(chrono.ChQuaternionD(1, 0, 0, 0))  
beam.SetFiniteElementType(chrono.ChBeamElementTypeHEXA)  


simulation.Add(beam)


class CustomMotor(chrono.ChFunction):
    def __call__(self, t):
        return chrono.ChVectorD(0, 0, 100 * np.sin(t))

motor = CustomMotor()
force = chrono.ChForce()
force.SetForceFunction(motor)
force.SetPoint(chrono.ChVectorD(0, 0, 1))  
beam.AddForce(force)


constraint = chrono.ChConstraint()
constraint.SetBody(beam)
constraint.SetPoint(chrono.ChVectorD(0, 0, 0))
constraint.SetScrew(chrono.ChMatrix33D().Get_identity())
simulation.Add(constraint)


while vis.Run():
    simulation.DoStepDynamics(0.01)  
    vis.Render()  

vis.Detach()