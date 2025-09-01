import math as m  # Import the math library for trigonometric functions, constants, etc.
import pychrono as chrono  # Import the main PyChrono library
import pychrono.fea as fea  # Import the finite element analysis module from PyChrono
import pychrono.mklsolver as mklsolver  # Import the Pardiso solver module from PyChrono
import pychrono.irrlicht as chronicls  # Import the Irrlicht visualization module from PyChrono
import os  # Import the OS library for file and directory operations

# Custom function class for motor angle:
class ChFunctionMyFun(chrono.ChFunction):
    def __init__(self):
        super(ChFunctionMyFun, self).__init__()
    def GetVal(self, x):
        if x > 0.5:
            return chrono.CH_PI
        else:
            return -chrono.CH_PI * (1.0 - m.cos(chrono.CH_PI * x / 0.3)) / 2.0

# Define the output directory path
out_dir = chrono.GetChronoOutputPath() + "BEAM_FAILED"

# Create a Chrono::Engine physical system
sys = chrono.ChSysSMC()

# Define key geometrical parameters
L = 1.2
H = 0.4
K = 0.07
vA = chrono.ChVector3d(0, 0, 0)
vC = chrono.ChVector3d(L, 0, 0)
vB = chrono.ChVector3d(L, -H, 0)
vG = chrono.ChVector3d(L - K, -H, 0)
vd = chrono.ChVector3d(0, 0, 0.0001)

# Create a truss body, fixed in space:
body_trss = chrono.ChBody()
body_trss.SetFixed(True)
sys.AddBody(body_trss)

# Attach a visualization shape to the truss
boxtruss = chrono.ChVisualShapeBox(0.03, 0.25, 0.15)
body_trss.AddVisualShape(boxtruss, chrono.ChFramed(chrono.ChVector3d(-0.01, 0, 0), chrono.QUNIT))

# Create a crank body:
body_crank = chrono.ChBody()
body_crank.SetPos((vC + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a visualization shape to the crank
boxcrank = chrono.ChVisualShapeBox(K, 0.05, 0.03)
body_crank.AddVisualShape(boxcrank)

# Create a rotational motor
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(body_truss, body_crank, chrono.ChFramed(vG))
myfun = ChFunctionMyFun()
motor.SetTorqueFunction(myfun)
sys.Add(motor)

# Create a FEM mesh container:
mesh = fea.ChMesh()

# Define horizontal beam parameters
beam_wy = 0.12
beam_wz = 0.15

# Create section properties for the IGA beam
minertia = fea.ChIneritaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)
melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(72.0e9)
melasticity.SetShearModulusFromPoisson(0.35)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)
msection1 = fea.ChMassSectionCosserat(minertia, melasticity)
msection1.SetDrawThickness(beam_wy * 0.5, beam_wz)

# Build the IGA beam
builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh, msection1, 30, vA, vC, chrono.VECT_X, 3)

# Fix the first node of the horizontal beam
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[65]
node_mid = builder_iga.





,
,
,
































,
,



























y,













,
,






_































_


















_








1_e_



































































































































































































































































































































































































,

,
,





,













































,


































































































































































































_

pre




s111






_s
s,


s
1()















































































































































111


























































































s





































































































,



,


























,





























,
,





































0.

















,





















.









,





,











,



























































s,










































































































































































































































































































































,














s.
C,
,
,




,
,

,

,



,
,
,


,
















,
,



,

,


,
,


,


,
,


,



,
,
,























,
















e












e



.





















,
,
,

































































_1







































































































































































































0















































































,
,





















































































m,


,


























































































































































































































































11,c,

























s,











s

























c,









e,

























































1



#










c




#
#
#


#


#

#
#


















1





#6
be,be,
,

be,1,1



4
1





















c()












#
##
c #c,
,be,
,s,



























)
h)
)
m,
d,d)
)
)
)


,


















x)
#


#m,ch)
)



)
)
)

)







)







1)


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

0,





0)










s_





0atat_







beat)



s,0)
)0)
)
)
0,c)
)
x)


)
)




,

be)



be)
)
6)c)c)
)c)
c,s,for_c,
,s)





c,6,6,for_



)0)
c,




c,c)

c,c,1,c,c,c,c,
,
)
1,1,11
1.s,c,1,c,c,c,c,ch)
,
,c,for.
1,1,1,1)
1)
0,0,1.1110,1.1.11,11,1,1,1,1,


1,h,1,1,c,1,1)1)
1,s,ch,m,c,
h,





ch,0,s,s,
a,6)



s,1)
)


























1,



0














,
,
,
,




,








,









111
















11






























,











,





























,
,
,
























,



















,

,





,
,





,
,








































,







,





















,
,
,




1,1,





























1,






s.c







,
,



,s,


,s,




,s,e,


,s,



s



,
,

,
,
,

,
,

,
,


c,
,





,
,



,
,


,
,
,
,
,1,1




center,
,
,



,




















,





,
































,





























































































c



1
c




1_s




s



,







be

s1





1







1,



c
c,c,c)


c
c,c,c_c
c1



#










1,1,


,
,c,
,
,c,


1



s

#


a















c,c,
center,


,center,2,
,
,
1,






1
111




























1




1
6



1



















1
6














1
1

























































1


211

be4
11,






,
,


x_ch_1_1_ch,c,c,
,

1,1,e,
,
,
,11,1,1111,

,1,c,



c,b,b,c,
1,1,1,b,1,


,from,be,


,
to_c)
,




1_to_s,

,1,be,b,

s,s,


s,w,s1,1,s1)

222


at,c,s,
c,
1,
with,with,s,toto,s1,
at
toat,s
0
)
s

to

,
,
to,
s,s,

s


s



s
s
s
s
s
s)
s









,
,
s,
,
,be,s,
,

to # #
#tototo


s
c
c_to_to,c,c,to_to_c,c_be)
s
s
s)
)
s
s



s,c)s)
s)c)
c)
c)
c,s)c))s)
s)
c)c)c)
)
)c)c)
s)
c)


)
c)
,ch,c)c)
ch)
)
)c)



1)s)

)
)
)


)
)
c)
)
)

)
center)
ch)with,d)
)ch)ch)ch)
)
)ch)
)
)
)











center,center)




center)
center)



s






s)


)
0)
)




w)
)
center)





1)


s)
)
)ch)ch)
)
)
)
)
)
)
c1)
0)1)


c)c)
c)11s)
)
)
))
)












11c0



c,c,
)
)


1,111)
1,c
c)
c1,c,c,ch,s,c,c,c11111,1,c,c,s,c)c)c)c,0,c,c,c,c,1,c,center1,1,c,c,c,c,c,c,c,c,c,1))
,111,1,1,1,to,center,c,c,1,
,s,
)
)
)
)
1,1111)
))
)
,s,)
)
1)1)1)11)11111,1,c,c,1,s)
1)1)
)c111)c)c)1)center)11))))
)
)
)
)1)11,c,c)c,c)ch)s)
)
)
)
)
)
)
)1)g)c06,c)
)c)c)c)
,c,c)
,s,c)c,m,m)mes,m,c,c)))
)
)
)c,c)
)
)
)c)c)c)c)
)c)c))))
,c)
c,c,m,m)
)c)c)
)
)
)
)
)m)
)
)
)
)
)
)
)
)
1)
)



)
)
)
)
)
)
)
m)
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
a





)
)
)
)
)
))))
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
c)
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



)m)
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


c)
cat)
)
)
c)m)1)1)
)
)














s.



,
,


,s,to,to,11,1,1,to,
1,1,c,to,m,m,
,
,
,to,
,_1,1,to,c,m,1,
,s,c,1,1,
,))))11)1)
)
,d,c,d,m,c,1,
,1,)
,1,11,11111,1,1,1,1,c1,1,1,1,11,c,_,c1)
,1,1)




1













1.111111111,11






,


111,1,

11()


1)











11,11111

c
c11)
c11at










1111111
11




1


1
c,
,

1
1)


11,m,c,m,d)
,c,c,
,c1111

111111
111111,c,c


m1c





c1

111111,111










1








111

1111
1
1



111,

1111



11


















































1






























11

























1
1
1111

























1
)









11

1


1_111
























1,1_11
1()111111

1
1










1_
























m_d_






















,

em,m,

,
,















,
,
,1,1,
1,



m,




,



)
m,



,m,






1,1,


1,

,






,
,1
1,11r11
m11)


m,


1,1
c




center_1_m1_1



1_c_s_c_c_

1_c_cm
m




c



m_










c

c
c,c
m,m,m




m,m,
,
,
m,
,
m1,















c



c_c_1_cm,1,1,





1









c



c,c,c,c,c,c,











1,1,

1,1,1,111,
1,111,1,1,1,1,



1,








































1,c,









111_1






















































































cr






















1

1
1
1_m

















































































,




s
















,















,






























,











,




1












,



1,











1_





center_



c_s_c_
,c,

1_
print("error happened with only start ```python")