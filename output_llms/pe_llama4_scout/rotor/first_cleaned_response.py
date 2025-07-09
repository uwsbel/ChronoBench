import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath('/path/to/data')


system = chrono.ChSystemSMC()


system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Jeffcott Rotor Simulation')
vis.Initialize()


beam = fea.ChBeamIGA()


beam.SetLength(1.0)  
beam.SetSection(chrono.ChBeamSectionEulerAdvanced())
beam_section = beam.GetSection()
beam_section.SetYoungModulus(200e9)  
beam_section.SetShearModulus(80e9)  
beam_section.SetDensity(7850)  


system.Add(beam.GetBody())


flywheel = chrono.ChBody()


flywheel.SetMass(10)  
flywheel.SetInertiaXX(chrono.ChVector3d(1, 1, 1))  


flywheel.SetPos(chrono.ChVector3d(0.5, 0, 0))  


system.Add(flywheel)


motor = chrono.ChLinkMotorRotationSpeed()


motor.Initialize(beam.GetBody(), chrono.ChBody(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))


motor.SetSpeedFunction(chrono.ChFunction_Const(10.0))  


system.Add(motor)


beam_shape = chrono.ChVisualShapeBox(0.1, 0.1, 1.0)
beam.GetBody().AddVisualShape(beam_shape)

flywheel_shape = chrono.ChVisualShapeSphere(0.2)
flywheel.AddVisualShape(flywheel_shape)


while vis.Run():
    system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

def main():
    
    chrono.SetChronoDataPath('/path/to/data')
    system = chrono.ChSystemSMC()
    system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(800, 600)
    vis.SetWindowTitle('Jeffcott Rotor Simulation')
    vis.Initialize()

    
    beam = fea.ChBeamIGA()
    beam.SetLength(1.0)
    beam.SetSection(chrono.ChBeamSectionEulerAdvanced())
    beam_section = beam.GetSection()
    beam_section.SetYoungModulus(200e9)
    beam_section.SetShearModulus(80e9)
    beam_section.SetDensity(7850)
    system.Add(beam.GetBody())

    
    flywheel = chrono.ChBody()
    flywheel.SetMass(10)
    flywheel.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
    flywheel.SetPos(chrono.ChVector3d(0.5, 0, 0))
    system.Add(flywheel)

    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(beam.GetBody(), chrono.ChBody(), chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI / 2)))
    motor.SetSpeedFunction(chrono.ChFunction_Const(10.0))
    system.Add(motor)

    
    beam_shape = chrono.ChVisualShapeBox(0.1, 0.1, 1.0)
    beam.GetBody().AddVisualShape(beam_shape)
    flywheel_shape = chrono.ChVisualShapeSphere(0.2)
    flywheel.AddVisualShape(flywheel_shape)

    
    while vis.Run():
        system.DoStepDynamics(0.01)
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

if __name__ == "__main__":
    main()