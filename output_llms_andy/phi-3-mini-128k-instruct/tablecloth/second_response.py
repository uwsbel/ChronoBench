```python
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory setup
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create Chrono physical system
sys = chrono.ChSystemSMC()

# Create and add mesh to the system
mesh = fea.ChMesh()
sys.Add(mesh)

# Material properties
density = 100
E = 6e4
nu = 0.0
thickness = 0.01

# Create material
melasticity = fea.ChElasticityKirchhoffIsotropic(E


# Create visualizations for shell elements
mvisualizeshellA = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellA.SetShellResolution(2)
mvisualizeshellA.SetMaterial(material)
mvisualizeshellA.SetColor(1.0, 0.0, 0.0)
mesh.AddVisualShapeFEA(mvisualizeshellA)

mvisualizeshellB = chrono.ChVisualShapeFEA(mesh)
mvisualizeshellB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT)
mvisualizeshellB.SetSymbolsThickness(0.006)
mvisualizeshellB.SetMaterial(material)
mvisualizeshellB.SetColor(0.0, 1.0, 0.0)
mesh.AddVisualShapeFEA(mvisualizeshellB)

# Create node monitoring and loading setup
nodePlotA = chrono.ChNodePlot()
nodePlotA.SetColor(1.0, 0.0, 0.0)
nodePlotA.SetLabel("Monitor A")
nodePlotA.SetMarker(chrono.ChNodePlot.Marker_Cross)
nodePlotA.SetLineWidth(2.0)
nodePlotA.SetPenWidth(2.0)
nodePlotA.SetPenColor(0.0, 0.0, 0.0)
nodePlotA.SetPenStyle(chrono.ChNodePlot.PenStyle_Solid)
nodePlotA.SetPenThickness(2.0)

nodePlotB = chrono.ChNodePlot()
nodePlotB.SetColor(0.0, 1.0, 0.0)
nodePlotB.SetLabel("Monitor B")
nodePlotB.SetMarker(chrono.ChNodePlot.Marker_Circle)
nodePlotB.SetLineWidth(2.0)
nodePlotB.SetPenWidth(2.0)
nodePlotB.SetPenColor(0.0, 0.0, 0.0)
nodePlotB.SetPenStyle(chrono.ChNodePlot.PenStyle_Solid)
nodePlotB.SetPenThickness(2.0)

nodesLoad = []

# Create elements for monitoring
ementmonitor = None
for iz in range(nsections,


,



)
)
)
)

)
)





)













0,


0.
)
))
)
)
)
)
)
)
)
)

)


)
)
)
)
,,)
)
,
25)

)
)
)
)
)
)
)
)
)
)
)
)
))
)

)

)
)



)
)



)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)





























)














































,

)
























































,
,
)

)
)
)




,
,
,
,
,
,0,

, 
)
)
)










,
,
)

,
)
,
)

,

)
,
,





)
)
)
,
)

,
,
)
)
)


)
)


)
,
)
)


)
























)
)

)


,
)
)





























)















































,
,




















































































,
































,
,






































all)


,
,
,
,
,






















,
,
,0,

)
)


)
,
,
)


)
)



























)
)







)

)
)








)

)
)



)




















)




)
)





)
)
)
)
)


)


















00









)
)
)
)


,
)



















,
,
, ,
,
)
)0)
1,
)
)
)
)
)
)


)
)
)


,
,
,
,
,
,
,
))
)
)
)
)
)
)0)
)
)
)
,



,
,
,)
)
)
,
,
)
)
)
)





,



)
)
)
)

,



)
,
)
)







,
,
)




)





)





)




)
)
)
)
))
)





,

)
)
)
)
)
,
,
)
,
,),)
,)
)
,)
)
)
)
,
,

,
,


,)
,
,


,
,)
2,0)
)))))))))))))))))))))
)
)
)
)
,)
,
,
,
)
)
)))))))),)
)))))
))))))
))
,)
)
)
)

)
)






)
)
)
)
)
)
)
))
)
)))
)
)
)
,
,)
)
)
)
)

)
)
)
)
,)
)
.





)
,
)
)


















)





)
)
)
)









)
)




,
)
)
,
)
)
)







)
)
)
)
)




,
















)
)
)
)
)
)
)



)
)
)
)
)

)
)
)
)
)
)
)
)
)
)
)
)
)

)
)
)
)
)
)
)
)
)
)
)
,
,
)
)
)
)
)
)
)
)
)
))
)
)
)
)
)
)))
)
)
)
)
)
))
)
)
)
)
)
,
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)

)
)
)
)









,











)
)
)
)
)
)



)
)












)

)
)
,
,)
)



)
)





)




)



)













































,


,






,
,
,
,





,


0
,
0,0)
)
,
,))
)
)
)
)
)
)
)
,

)
)
,




,
,

,
,
,
,
,0,
,
,
,
)
)
)
)
)
)
,
,
)










,
,
,

)
)
,

)
,
)
)
)




)



)

)




)
)
)

)
.)
)
)


)
)
,
,)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)










)
)




)
,
,
,
)
)

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
)
)




,0,
,
)
,0,




)
)
,
,
,
,
,

















,
,
,
00,)


)

)




)

)
)
)









,
,
)
)

,







)
)
)





,




















)
)






0
)


)
)
)




































)

















)
)
)
)



)


















































)




















3

,
,

)
)
)

)





)

)



)
)









,
,
,
,
,



)
)
)
)
)))))))
))
)
)



)
)
,
,))))
))
,)
)
)
)
,
,












)
)
)
)
)
)
)
)
)
















)
)
)





)









)
)
)









)












)
)
)
)
)
)

)





)





































































,
,



,

,




0












,

0)
,




,
,
,
,0,0,
,

,
,


,
,
:













,




,
,
,
,


,



,
,



)
)
)
)






,

)
,
,



,
,


)




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
)





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

,0,
,




,
,
,
,
,








,
,
)
)
,)
)
)
,
,


,





,





,





,
,

,)
,
)
)
)




)


)
)



















,



)
,

)
)












)
)
)





)
)













)
)



)
)
)



)
)







)





)










,



)


)















)
,)
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
,)
,
,
,2,,)
)
,)
))
)
)
)
)
)
,)
)
)
,
,
,
,0,0,)2)2)
)
)
)
)
)
)
)
))
)
)
)
)
)
)
)
)
)
,)
)
)
,))
)
)
)
)
)
))
)
)
)
)
)
)
)
)
)
)
)
)
,)
)
)
)
)
)
)
)
)
)
)
)
)







)
)
)
)
)
)
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
,)
)
)
)
)
)




,
)

)







,
,
,0,0,0,
,2,
)
)
)
)
)
)





)





)


,
,
,
,
)
,)
)

)
)

)
)

)
)
)


)
)



)
)





,



)
)
)
)
)




,














,

)
)



)







)






























)
)



)




)











)

.



)










)
)
















)
)











)

















)
)

)
)
)





,
,
,



)
)
)
)
)
)



)
)
)
)







)
)
)
)




,
)
)

)



























)




)
,









































)













































































3


















3









































)
)

















,



)
)

)
)
)







)





)
)
)


)
))
)
)))))
)
)
)
)



)






)
)))
)
)
)
)
)
)
)
)
)
)
)
)
)
)
)

)











)
)
)
)
.
)
)
)
)
)

)
)





















)
)













































,
,


























)

)














































)
,
,









































,
,
















)



















,




)
)











































































3)

3


)


























































)
)











)
3)


)
)

)



























)

)
)
)
)
)
)
)
))


















)
)








)
)
)
















)
)


)



,
)
)


)
)
)
)

















)

)
)

)
)
)


































)













































































,











,
,








,






























2,



















,
,



,


,
,
,
,
,
,
,

,
,




3,


2,




,





,













,





















,
,
,

,
,
,
,
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





































,3,2,

3













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

)

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



)
)
)



















































































.

























,






,


,

























)
,



,































,
,
,


)
)
)
)
,
)
)




















)
)
)
)
)
)
)
,


)

)
)

)



)
)







)
,


,
,)
,
,)

)


)
)
,
,



)
)



))
)

)


3,

)
,
,



3






















,
)
)


)
)








































)











































,

3





3,

,



3,







3,
3,0,
,0,0,

,


















,




,
,



,
,
0,


,


,

,
,
,
,
,
,
,
,

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

)





















,
,













,

)













,
,

























)
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
)
)
)
)
,
)
)
)
,



,
,
,

,
,)
)
)
,
)
,
,)


,
)











,
,


,


,
,
,
3,







,
,


)


















































3
3





,


















3,































































3,
,










3








































































































)
,







































,
,















































)
,



























,











,
























































































0

















:
:



:


,
:1:

,











,
:
:
:
:
:0:0:2:6,
, , ):

2,3,





























,0,0,0,1,
, 
3,




,



,



















,





,



,
,




,
,
,











,



,





,


,
,
,









,












,



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
)
,
,
,3,
,
,










,
,
,
,
,
,


,
,
,
,
, 



,
,

,
,

,3,
,
,
,

,
,
)



3,

,
,
,
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
,)

)
)
)
)
b,
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

)



)



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







3


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
0)
)
)
)
)
)
)
)
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
,)
)
)
)
)
)
)
)
,

)
)
)



,
,
,

,
,
)
,

)

,
,
)
)
)
)
)
,
,
)
)
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
)



,

,
,







,

,
,
,
,3,
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
,3,




,
,

,


,
,
, 
,
,
,
,
,
,
,
,










,
,
3,
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




)


















)




,
,
,


,
)
)
)
,
,







0,
,
)
)
)
)
)
)
)
)
,)
,)
)))
)
)))
)
)))
)
)
)
)
)
)
)
)
)
)
)
)
))
)
)))
)
)
)
,
,



,


)

)


)
,
,


,

)
)
)
)
)
)
)









)
)






)






,
,
,
,3


,
,
,
,



































































,






























3
,

,
,
)






















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
3,



,
,
,
,
,
,

,
,
,


,

,




,



,






,
,
,
,


,

  
, 
,
,
,
,
,






,
,










,





































,













































,






































,
,



,
,
,






,




















,
,
,
,
,
,











,
,
,
,
,
,
,
,
,
,
,
,
,
,




,
,3,






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











































































































3,
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
,                ,3, 
,
,0)
,
,
,
,
,




,


,
,
,
,
,
,
,
,
,
,
,
,
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
,1,
, ,
, #,
,4, #, )
, ,2,
, ,
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
,3,
,
,
,
,
,
,
,
,
, 3,
,
,
,0,

,
,
,

,
,


,
,
,
,
,
,















,
,








,

,
,
,


,

,








,
,
,
,
,

,
,

,
,
,
,
,
,
,





















,
,
,








,
,

,
,


















,
,
,
,
,
,
,
,
,
,
,
,
















,
,






,
,0)
,)
,))
)
)
)



,
,
,
,
,
,



,
,
, 1,)
,
,
, ,
,
,
,
,

,
,


,
,
,
,
,
,
,
,
,
,
,
,
,

,
,




,
,
,
,
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



b.0,
,
,
,
,0,
,
,
,
,



,
,
,
,
,
,
,
,
,
,
,
,
,
,
,




,





,
,
,
,
,

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
.

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


,
,
,
,
,
,
,
,














,



,


,
,
,
,
,
,
,
,










































,
































,






































,

,


,
,


,








,
,
,
,
,
,
,
,
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
,
,
,
,
,
,



,
,












,
,
,
,
,
,
,
,
,
,
,




















,
,
,


,
,






,


,
,
























,
,
,

,



























,


,
,
,

,
,























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























)
)
)
)
)















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






)
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








)
)
,








,














































































































































































































































































































)











































































































































































































)











































































































































































































































































































































,
,


























































































,





















































































































































































,

)
)
)
)








,

































)









)
,
,
,

































































































































































































































































































3.

























































































































































































































































3,,





































0,0,



























,
,



,




,




,





































,



,

















































































































)
)
)

)





0)


















,










0.