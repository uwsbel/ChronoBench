import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr







class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        for i in range(n_chains):
            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  

            
            builder = fea.ChBuilderCableANCF()

            
            num_elements = i * 2 + 10  
            builder.BuildBeam(
                mesh,  
                msection_cable2,  
                num_elements,  
                chrono.ChVector3d(0, 0, -0.1 - 0.1 * i),  
                chrono.ChVector3d(0.5 + 0.1 * i, 0, -0.1 - 0.1 * i)  
            )

            
            
            builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -0.7, 0))  

            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(builder.GetLastBeamNodes().back(), mtruss)
            system.Add(constraint_hinge)  

            
            box = chrono.ChBodyEasyBox(0.1, 0.1, 0.1)
            box.SetPos(builder.GetLastBeamNodes().back().GetPos())
            system.Add(box)

            
            constraint_hinge_box = fea.ChLinkNodeFrame()
            constraint_hinge_box.Initialize(builder.GetLastBeamNodes().back(), box)
            system.Add(constraint_hinge_box)

        
        def PrintBodyPositions(self):
            for i in range(self.n_chains):
                print(f"Chain {i+1} end body position: {system.GetBody(i*2 + 10 + 1).GetPos()}")

    


sys = chrono.ChSystemSMC()
mesh = fea.ChMesh()


model = Model1(sys, mesh)
sys.Add(mesh)