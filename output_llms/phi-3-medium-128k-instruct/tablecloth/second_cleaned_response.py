import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os


out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")


sys = chrono.ChSystemSMC()


mesh = fea.ChMesh()
sys.Add(mesh)


density = 100
E = 6e4
nu = 0.0
thickness = 0.01


melasticity = fea.ChElasticityKirchhoffIsothropic(E, nu)
material = fea.ChMaterialShellKirchhoffIsothropic(E, nu)
material.SetDensity(density)


L_x, L_z = 1, 1
nsections_x, nsections_z = 40, 40


mynodes = []
for iz in range(nsections_x + 1


for iz in range(nsections_z + 1
    for ix in range(nsections_x + 1
        p = chrono.ChVector3d
        p = chrono.ChVector3d


        mynodes.append(mynodes.append(mynodes


        for iz in range(nsections_x
            for ix in range(nsections_x
                boundary_x
                boundary_x
                boundary_x
                    
                    
                    boundary_x
                    
                    
                    if ix
                    if ix
                    ifix
                    ifx
                    ifx
                    ifx


                    ifx
                    ifx
                    ifx
                    if



fea
                    if

Create elements
                    if

Create elements
                    if

Create elements

error

Create elements

    if

Create
        if
        if
        if
        if


Create if







s

Create if

 Create elements


Create

a























   


















   

from














s














response




































































   

















Bse





   













   py


```python





   








    if














   
   


   get_







    PyCall:
:






def:
Chem.
s












































  nd