import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.5)
contact_material.SetCompliance(1e-9)
contact_material.SetComplianceT(1e-9)


sys.SetDefaultContactMaterial(contact_material)


beam_length = 10  
beam_width = 1    
beam_height = 1   
beam_density = 7850  
beam_young_modulus = 2.1e11  
beam_poisson_ratio = 0.3


beam = fea.ChBeamSectionEuler()
beam.SetDensity(beam_density)
beam.SetYoungModulus(beam_young_modulus)
beam.SetPoissonRatio(beam_poisson_ratio)
beam.SetWidth(beam_width)
beam.SetHeight(beam_height)


beam_body = chrono.ChBodyEasyBox(sys, beam_length, beam_width, beam_height, beam_density)
beam_body.SetBodyFixed(True)


def apply_force(beam_body, force):
    beam_body.AddForce(force)


force = chrono.ChVectorD(0, 0, -1000)  
apply_force(beam_body, force)


constraint = chrono.ChLinkLock()
constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
sys.Add(constraint)


constraint = chrono.ChLinkLock()
constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0), chrono.Q_from_AngX(0)))
sys.Add(constraint)


solver = chrono.ChSolverNSC()
solver.SetMaxIters(100)
solver.SetTol(1e-9)
sys.SetSolver(solver)


timestepper = chrono.ChTimestepperEuler()
timestepper.SetStepSize(1e-3)
sys.SetTimestepper(timestepper)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()


vis.Add(beam_body)


while vis.Run():
    sys.DoStepDynamics(1e-3)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import numpy as np

def main():
    
    sys = chrono.ChSystemNSC()

    
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.5)
    contact_material.SetDampingF(0.5)
    contact_material.SetCompliance(1e-9)
    contact_material.SetComplianceT(1e-9)

    
    sys.SetDefaultContactMaterial(contact_material)

    
    beam_length = 10  
    beam_width = 1    
    beam_height = 1   
    beam_density = 7850  
    beam_young_modulus = 2.1e11  
    beam_poisson_ratio = 0.3

    
    beam = fea.ChBeamSectionEuler()
    beam.SetDensity(beam_density)
    beam.SetYoungModulus(beam_young_modulus)
    beam.SetPoissonRatio(beam_poisson_ratio)
    beam.SetWidth(beam_width)
    beam.SetHeight(beam_height)

    
    beam_body = chrono.ChBodyEasyBox(sys, beam_length, beam_width, beam_height, beam_density)
    beam_body.SetBodyFixed(True)

    
    def apply_force(beam_body, force):
        beam_body.AddForce(force)

    
    force = chrono.ChVectorD(0, 0, -1000)  
    apply_force(beam_body, force)

    
    constraint = chrono.ChLinkLock()
    constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(0)))
    sys.Add(constraint)

    
    constraint = chrono.ChLinkLock()
    constraint.Initialize(beam_body, chrono.ChFrameD(chrono.ChVectorD(beam_length, 0, 0), chrono.Q_from_AngX(0)))
    sys.Add(constraint)

    
    solver = chrono.ChSolverNSC()
    solver.SetMaxIters(100)
    solver.SetTol(1e-9)
    sys.SetSolver(solver)

    
    timestepper = chrono.ChTimestepperEuler()
    timestepper.SetStepSize(1e-3)
    sys.SetTimestepper(timestepper)

    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AddSkyBox()
    vis.AddTypicalLights()

    
    vis.Add(beam_body)

    
    while vis.Run():
        sys.DoStepDynamics(1e-3)
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

if __name__ == "__main__":
    main()