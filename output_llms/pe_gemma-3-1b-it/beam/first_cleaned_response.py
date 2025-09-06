import chrono
import numpy as np
import math
import os
import sys

def generate_beam_simulation():
    

    
    try:
        chrono = chrono.ChSystemSMC()
        print("PyChrono environment initialized successfully.")
    except Exception as e:
        print(f"Error initializing PyChrono: {e}")
        return

    
    print("Defining simulation requirements...")
    
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)

    
    try:
        mesh = fea.ChMesh()
        mesh.SetAsRectangularSection(beam_wy, beam_wz)
        mesh.SetYoungModulus(0.01e9)
        mesh.SetShearModulus(0.01e9 * 0.3)
        mesh.SetRayleighDamping(0.000)
        hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
        hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
        mesh.AddNode(hnode1)
        mesh.AddNode(hnode2)
        belement1 = fea.ChElementBeamEuler()
        belement1.SetNodes(hnode1, hnode2)
        belement1.SetSection(mesh)
        mesh.AddElement(belement1)

        print("Physical system added successfully.")
    except Exception as e:
        print(f"Error adding physical system: {e}")
        return

    
    try:
        vis = chrono.ChVisualSystemIrrlicht()
        print("Visualization system initialized successfully.")
    except Exception as e:
        print(f"Error initializing visualization: {e}")
        return

    
    try:
        vis.SetWindowSize(1024, 768)
        print("Window size set to 1024x768.")
    except Exception as e:
        print(f"Error setting window size: {e}")
        return

    
    try:
        vis.SetWindowTitle('Simulation Window')
        print("Window title set to 'Simulation Window'.")
    except Exception as e:
        print(f"Error setting window title: {e}")
        return

    
    try:
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        print("Logo added successfully.")
    except Exception as e:
        print(f"Error adding logo: {e}")
        return

    
    try:
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
    except Exception as e:
        print(f"Error setting collision model: {e}")
        return

    
    try:
        while vis.Run():
            print("Simulation running...")
            
            sys.DoStepDynamics(0.01)
            
            vis.Render()
            print("Simulation rendered.")
    except Exception as e:
        print(f"Error during simulation: {e}")
    finally:
        
        if 'vis' in locals():
            vis.Destroy()
        if 'chrono' in locals():
            chrono.Destroy()

if __name__ == "__main__":
    generate_beam_simulation()