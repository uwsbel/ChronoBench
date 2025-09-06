import chrono
import numpy as np
import math
import os
import sys
import time


try:
    chrono = chrono.ChSystem()
    chrono.SetChronoDataPath('/path/to/data')  
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    sys.exit(1)


def run_beam_buckling_simulation():
    

    
    
    mesh = chrono.ChMesh()
    mesh.SetAsRectangularSection(beam_wy, beam_wz)
    mesh.SetYoungModulus(0.01e9)
    mesh.SetShearModulus(0.01e9 * 0.3)
    mesh.SetRayleighDamping(0.000)
    hnode1 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    hnode2 = chrono.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
    mesh.AddNode(hnode1)
    mesh.AddNode(hnode2)

    
    belement1 = chrono.ChElementBeamEuler()
    belement1.SetNodes(hnode1, hnode2)
    belement1.SetSection(mesh)
    mesh.AddElement(belement1)

    
    motor = chrono.ChLinkLockLock()
    motor.Initialize(belement1, hnode1, chrono.ChFramed())
    chrono.ChSystem.Add(motor)

    
    beam_wy = 100  
    beam_wz = 100  
    beam_L = 100  
    beam_M = 100  
    beam_D = 100  
    beam_N = 100  
    
    
    chrono.ChCollisionSystem.SetDefaultSuggestedEnvelope(0.01)
    chrono.ChCollisionSystem.SetDefaultSuggestedMargin(0.005)
    chrono.ChCollisionSystem.SetDefaultSuggestedStrength(0.01)
    chrono.ChCollisionSystem.SetDefaultSuggestedForce(0.0)

    
    initial_position = chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT, beam_wy, beam_wz)

    
    while True:
        
        sys.DoStepDynamics(0.01)
        
        
        vis = chrono.ChVisualSystemIrrlicht()
        vis.AttachSystem(chrono)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle('Beam Buckling Simulation')
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png')) 
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(0, 0, 0))
        vis.SetCameraPosition(chrono.ChVector3d(0, 0, 0))
        vis.SetCameraRotation(chrono.ChVector3d(0, 0, 0))

        vis.Render()

        
        print("Simulation running...")
        time.sleep(0.1)  
        print("Simulation finished.")

    
    chrono.ChSystem.Shutdown()


if __name__ == "__main__":
    run_beam_buckling_simulation()