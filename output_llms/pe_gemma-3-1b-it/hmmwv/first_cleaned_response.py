import chrono
import pychrono
import math
import numpy as np
import os
import time


chrono = chrono.ChSystem()
chrono.SetChronoDataPath('/path/to/data')  


try:
    chrono.ChSystemNSC()
    chrono.ChSystemSMC()
    chrono.SetChronoDataPath()
except Exception as e:
    print(f"Error initializing Chrono system: {e}")
    exit()


try:
    material_nsc = chrono.ChContactMaterialNSC()
    material_smc = chrono.ChContactMaterialSMC()
    material.SetFriction(0.5)
    material.SetDampingF(0.1)
    material.SetCompliance(0.01)
    
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
except Exception as e:
    print(f"Error setting visualization: {e}")
    exit()



try:
    body = chrono.ChBody()
    body.SetPos(chrono.ChVector3d(0, 0, 0))
    body.SetMass(100)
    body.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
    body.SetFixed(True)
    body.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    mesh = chrono.ChMesh()
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(msection)
    mesh.AddElement(belement1)
    
    
    rev_joint = chrono.ChLinkLockRevolute()
    rev_joint.Initialize(body1, body2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
    rev_joint.SetSpeedFunction(chrono.ChFunctionSine(0.001, 1.5))
    
    spherical_joint = chrono.ChLinkLockSpherical()
    spherical_joint.Initialize(body1, body2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT))
    
    univ_joint = chrono.ChLinkUniversal()
    univ_joint.Initialize(body1, body2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
    
    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(body1, body2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
    
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(my_system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('HMMWV Simulation')
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sys.DoStepDynamics(0.01)
    vis.Run()
    vis.EndScene()

    print("Simulation complete.")

except Exception as e:
    print(f"An error occurred: {e}")
    exit()