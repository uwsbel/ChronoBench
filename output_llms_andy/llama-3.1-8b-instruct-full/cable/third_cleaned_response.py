import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        
        msection_cable2 = fea.ChBeamSectionCable()
        msection_cable2.SetDiameter(0.015)  
        msection_cable2.SetYoungModulus(0.01e9)  
        msection_cable2.SetRayleighDamping(0.0001)  

        
        builder = fea.ChBuilderCableANCF()

        for i in range(n_chains):
            
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                3 * (i + 1),  
                chrono.ChVector3d(0, 0, -0.1 + i * 0.2),  
                chrono.ChVector3d(0.5, 0, -0.1 + i * 0.2)  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            box = chrono.ChBody()
            box.SetPos(builder.GetLastBeamNodes().back().GetPos())
            box.SetFixed(False)
            system.Add(box)

            
            constraint_box = fea.ChLinkNodeFrame()
            constraint_box.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_box)

    def PrintBodyPositions(self, system, step):
        bodies = system.GetBodies()
        for body in bodies:
            if body.GetFixed():
                continue
            pos = body.GetPos()
            print(f"Body {body.GetName()}: {pos.x}, {pos.y}, {pos.z} (step {step})")


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)  



visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)  
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)  
visualizebeamA.SetSmoothFaces(True)  
visualizebeamA.SetWireframe(False)  
mesh.AddVisualShapeFEA(visualizebeamA)  


visualizebeamB = chrono.ChVisualShapeFEA(mesh)
visualizebeamB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
print("error happened with only start ```python")