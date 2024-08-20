import pychrono.core as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr








class Model1:
    def __init__(self, system, mesh, n_chains=6):
        self.n_chains = n_chains
        self.chains = []
        self.end_positions = []
        self.system = system
        self.mesh = mesh
        self.create_chains()

    def create_chains(self):
        for i in range(self.n_chains):
            
            mtruss = chrono.ChBody()
            mtruss.SetFixed(True)  

            
            constraint_hinge = fea.ChLinkNodeFrame()
            constraint_hinge.Initialize(self.mesh.GetLastBeamNodes().back(), mtruss)
            self.system.Add(constraint_hinge)  

            
            msection_cable2 = fea.ChBeamSectionCable()
            msection_cable2.SetDiameter(0.015)  
            msection_cable2.SetYoungModulus(0.01e9)  
            msection_cable2.SetRayleighDamping(0.0001)  
            builder = fea.ChBuilderCableANCF()
            builder.BuildBeam(
                self.mesh,  
                msection_cable2,  
                i + 1,  
                chrono.ChVector3d(0, 0, -0.1 - 0.01 * i),  
                chrono.ChVector3d(0.5 + 0.01 * i, 0, -0.1 - 0.01 * i)  
            )

            
            mbox = chrono.ChBox()
            mbox.SetSize(chrono.ChVector3d(0.1, 0.1, 0.1))
            mbox.SetPos(self.mesh.GetLastBeamNodes().back().GetPos())
            self.system.Add(mbox)
            self.system.Add(constraint_hinge)

            
            self.end_positions.append(self.mesh.GetLastBeamNodes().back().GetPos())

            
            visualizebeamA = chrono.ChVisualShapeFEA(self.mesh)
            visualizebeamA.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)  
            visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
            visualizebeamA.SetSymbolsThickness(0.006)  
            visualizebeamA.SetSymbolsScale(0.01)  
            visualizebeamA.SetZbufferHide(False)  
            self.mesh.AddVisualShapeFEA(visualizebeamA)  

            
            visualizebox = chrono.ChVisualShapeFEA(self.mesh)
            visualizebox.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_BOX)
            visualizebox.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
            visualizebox.SetSymbolsThickness(0.006)
            visualizebox.SetSymbolsScale(0.01)
            visualizebox.SetZbufferHide(False)
            self.mesh.AddVisualShapeFEA(visualizebox)

    def PrintBodyPositions(self):
        for i, pos in enumerate(self.end_positions):
            print(f"Chain {i+1} end body position: {pos}")


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
visualizebeamB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)  
visualizebeamB.SetSymbolsThickness(0.006)  
visualizebeamB.SetSymbolsScale(0.01)  
visualizebeamB.SetZbufferHide(False)  
mesh.AddVisualShapeFEA(visualizebeamB)  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)  
vis.SetWindowSize(1024, 768)  
vis.SetWindowTitle('FEA cables')  
vis.Initialize()  
vis.AddLogo(chrono.GetChrono















































,,














































,











,














































,






























































































,

































,
















































,
























































,


,






,
,

,
,







,
,
,
,
,
,


,
,

,
,
,
,
,
,
,
,
,
,


,
,

,
,
,
,

,

,
,
,
,
,
,
,
,
,
,
,
,
,
,
,,,
,
,
,
,
,



,






,
,

,


,
,
,










,








,









,




,

,


,






,


















,










































































































































,
,
,









































,































,
,





















































































,



,

















































,
















,



































































,





,
,






,




,
,











,
,











,


,
,
,
,
,
,
,
,













 of












,




















































































,


















































,






,






,








,





,
,












,

















,





,






















































































,











































































































,






























































.



















,
,










,












































,





,



















,



,















































,
,
























































,























,
,,




,

























,
,
,
,


















,
























,

































































































































































































































































































,
,




,

















,















































































































,





,
,




,






























,
,








,
,
,

,





,
,)

,
,











,










,
,
,



,

































,




,




,
,

,
































,



,
,
,






















,
,

,
,
,






































,

,



















































































,























































































































,





,





















,



























































,
,







,










,








,
,







,




,

,
,
,


,
,
,
,
,
,




,
,

,
,

,
,
,



,
,



,
,

,
,
,
,
,
,
,
,
,

,
,,
,


,
,



,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,
,















,




,


,

,


,




,

,













,
,
,
,



















,




,

,









,
,

















,













,
,




,

















   





,















,
,




































,
,
,
,
,



,





,











,












,
,
,
,


,

,
,
,


,










,

,
,
,

,
,
,

,









,






,
,


,
,
,
,
,
,







,
,
,
,
,
,





,


,

,
,
,
,
,
,
,
,


,
,





















,

,
,












,
,
,
,
,
















,
,
,
,
,
,



,
,
,
,
,











,




,
,

















,


,
,













,
,





,

,
,
,
,,
,

















,
























,
,























,





,



































































































































































.
.




,
,


,
,
,
























,
,






























,





































































































,


,
,
,
,
,


.
























































































































,
,









,
,





























,
,





,








,































,
,




,
,




























,
,






























































































































,
,


























,









,
,










,



















,
,
,
,

































,







































,
,
,
,





,

















,
,
,
,
,
,


,
,
























,
,
,
,
,
,
,
,
,
,
,
,


























,
,
,
,
,
,


,
,

















,


,
,
,
,
,,,,
,,,,,,
,
,,






















,
,
,
,,
,
,




,





















,
,
,
,
,
,
,
,
,,,,,,,,














,





,

,
,
,
,


























,
,
,





,















,































,


























,
,
,
,
,
,
,







)
























,
,
,
,,,,
,
,
,





















,
,
,
,







,
,









































































,
,

,
,






































,

)

,
,














,
,
,

















,
,


,











,
,
,















,








,
,
,
,


,





,


,
,



,

,
,
,
























,
,
,
,








































,
,
,
,
,






























,





















































































,
,





























,
,






























,


















,























































































,
print("error happened with only start ```python")