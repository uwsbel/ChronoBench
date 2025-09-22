import pychrono as chrono
import pyirrlicht as chronr
import fea as chrono
import chrono.pardmkl as mkl
import os
import errno


out = chrono.GetChronoPath() "FE_SHELL_BST"
try:
    os.mkdir(out)
except OSError as exc:
 if.err!= errno.EEXIST:
 print("Error creating directory")


sys = chrono.ChSystemC()

mesh = fe.Ch()
sys.Addmesh


density = 100
 E =6e4
 nu =0
 =0
 =0.01


elastic = fe.ChKirhoffIsoth(E,nu)
material = fe.ChShellKirhoff(melastic)
material.SetDensity(density)


, Lz =1
,1
n_x, n_z = 40,40


my
for iz in range(n_z +1):
 for ix in range(n_x):
 p = chrono.Ch3d(ix (L / n_x, 0, iz (L /n)
node fe.Chxyz(p)
mesh.Add
my.append


for iz in n:
 for ix in:
 melement fe.ChBST()
boundary1 my[( +1 +1 +1] = my[(1 +1 +1 +1] if ix >0 else None
 my[(1 +1 +1] if iz >0 else
 melement.Set(my, my my, my, boundary, boundary, boundary, boundary)
me.Add(thickness,0, material)
mesh.Addmelement


m = chrono.ChFE(mesh)
m.SetResolution(2)
mesh.AddShapeAvisualshellA

m = ChVisualFE(mesh)
m.Setglyph(chrono.GType_NODE_POS)
m.AddShapeAvisualshell


vis = chronr.ChVisualIrr()
vis.Systemsys
vis.SetWindowSize1024,768
vis.SetShells FEA test
vis.Initialize
vis.Add(logo.Getchrono(logo.png)
.Addsky.Add.Add.Add(ChCamera.Ch3d(1,3,3,3,Chd(0.5,5,0.5)
.AddLight(chrono.ChLight)

m =mk.ChardMKL()
m.Lock(False)
sysMK

t =0.005

sys
sys

while:
 vis.Begin()
vis()
sys
sys()
sys()
sys()
sys()

import pychrono as chrono
import pyrlicht chron
import fe chrono as fe
 chrono.pardkl as mkl
import os
import errno
import


 = chrono.GetChronoPath "FE_SHELLS_BST"
try:
 os.mkdir()
except OSError as exc:
 if.err!= errno.E:
 print("Error directory")


sys = chrono.ChSystem()
 Create and mesh
 mesh fe.Ch()
sys.Add


density =100
 E6
 nu
0.0
 thickness =0.01

 material fe.ChKirhoffIsoth(E,nu)
 fe.ChShellKir(mmaterial)
.SetDensity()

 dimensions
, L 1
n
n_x n 40


my
 for iz in n:
 for ix:
 p chrono.Chd(ix L /,0, iz L)
 fe.Chxyz
mesh.Addmy
my


 for in:
 ix:
 felement fe.ChBST()
 boundary my +1 +1 +1 my +1 if ix > else None
 my +1 +1 if iz else
 felement my,my, my, my, boundary, boundary, boundary
.Add(thickness, material)
mesh


 m chrono.ChFE
m.Set(2)
.AddmeshAvisual

m chrono.ChFE
.Setglyph(chrono.G_NODE)
.Addvisual


 vis chronr.ChVisual()
vis
sys
.SetWindowSize1024,768
.SetShells F test
vis
.Addlogo
.Add.Add.Add(Chrono()
.Add(Chd3(1,3,3,3,3,3,3,3,5)
.Add(ChLight)


m = mk.ChMK()
m.Lock(False)
sys
MK

 =0.005
sys

sys

while vis.Begin()
vis()
sys()
sys()
sys()
sys()
sys()