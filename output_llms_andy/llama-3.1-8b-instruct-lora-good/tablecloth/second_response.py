import pychrono as chrono
import pyirrlicht as chronr
import fea as chrono
import chrono.pardmkl as mkl
import os
import errno

# Output setup
out = chrono.GetChronoPath() "FE_SHELL_BST"
try:
    os.mkdir(out)
except OSError as exc:
 if.err!= errno.EEXIST:
 print("Error creating directory")

# Create Chrono system
sys = chrono.ChSystemC()
# Create and mesh to the
mesh = fe.Ch()
sys.Addmesh

# properties
density = 100
 E =6e4
 nu =0
 =0
 =0.01

# material
elastic = fe.ChKirhoffIsoth(E,nu)
material = fe.ChShellKirhoff(melastic)
material.SetDensity(density)

# dimensions
, Lz =1
,1
n_x, n_z = 40,40

# nodes
my
for iz in range(n_z +1):
 for ix in range(n_x):
 p = chrono.Ch3d(ix (L / n_x, 0, iz (L /n)
node fe.Chxyz(p)
mesh.Add
my.append

# elements
for iz in n:
 for ix in:
 melement fe.ChBST()
boundary1 my[( +1 +1 +1] = my[(1 +1 +1 +1] if ix >0 else None
 my[(1 +1 +1] if iz >0 else
 melement.Set(my, my my, my, boundary, boundary, boundary, boundary)
me.Add(thickness,0, material)
mesh.Addmelement

# visualizationshell
m = chrono.ChFE(mesh)
m.SetResolution(2)
mesh.AddShapeAvisualshellA

m = ChVisualFE(mesh)
m.Setglyph(chrono.GType_NODE_POS)
m.AddShapeAvisualshell

# Irrlicht
vis = chronr.ChVisualIrr()
vis.Systemsys
vis.SetWindowSize1024,768
vis.SetShells FEA test
vis.Initialize
vis.Add(logo.Getchrono(logo.png)
.Addsky.Add.Add.Add(ChCamera.Ch3d(1,3,3,3,Chd(0.5,5,0.5)
.AddLight(chrono.ChLight)
# Change to PardMKL
m =mk.ChardMKL()
m.Lock(False)
sysMK
# Define step
t =0.005
# setup
sys
sys
# loop
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

# Output
 = chrono.GetChronoPath "FE_SHELLS_BST"
try:
 os.mkdir()
except OSError as exc:
 if.err!= errno.E:
 print("Error directory")

# Chrono
sys = chrono.ChSystem()
 Create and mesh
 mesh fe.Ch()
sys.Add

# material
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

# nodes
my
 for iz in n:
 for ix:
 p chrono.Chd(ix L /,0, iz L)
 fe.Chxyz
mesh.Addmy
my

# elements
 for in:
 ix:
 felement fe.ChBST()
 boundary my +1 +1 +1 my +1 if ix > else None
 my +1 +1 if iz else
 felement my,my, my, my, boundary, boundary, boundary
.Add(thickness, material)
mesh

# visualizations
 m chrono.ChFE
m.Set(2)
.AddmeshAvisual

m chrono.ChFE
.Setglyph(chrono.G_NODE)
.Addvisual

# Irrlicht
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

# Change to PardMK
m = mk.ChMK()
m.Lock(False)
sys
MK
# step
 =0.005
sys
# setup
sys
# loop
while vis.Begin()
vis()
sys()
sys()
sys()
sys()
sys()