import chrono
import numpy as np
import pychrono
import sys
import os
import math
import random

def create_tablecloth_simulation():
    

    
    try:
        chrono = chrono.ChSystem()
        chrono.ChSystemNSC()  
    except Exception as e:
        print(f"Error initializing Chrono: {e}")
        return

    
    try:
        mesh = chrono.ChMesh()
        mesh.SetAsRectangularSection(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(beam_L, 0, 0)) 
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

        
        material_nsc = chrono.ChContactMaterialNSC()
        material_nsc.SetFriction(0.5)
        material_nsc.SetDampingF(0.1)
        material_nsc.SetCompliance(0.01)

        
        vis = chrono.ChVisualSystemIrrlicht()
        vis.AttachSystem(chrono)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle('Tablecloth Folding Simulation')

        
        body = chrono.ChBody()
        body.SetPos(chrono.ChVector3d(0, 0, 0))
        body.SetMass(100)  

        
        rev_joint = chrono.ChLinkLockRevolute()
        rev_joint.Initialize(belement1, belement2, chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0)))
        rev_joint.SetSpeedFunction(chrono.ChFunctionSine(0.001, 1.5))
        
        while vis.Run():
            
            print(f"Time step: {chrono.ChTime.Now()}")
            print(f"Body mass: {body.GetMass()}")
            print(f"Body position: {body.GetPos()}")

            
            
            force = chrono.ChVector3d(0, 0, 0)
            force = force + 100 * chrono.ChTime.DeltaTime  
            body.SetPosition(body.GetPos() + force)

        
        print("Simulation finished.")

    except Exception as e:
        print(f"Error during simulation initialization: {e}")


if __name__ == "__main__":
    create_tablecloth_simulation()