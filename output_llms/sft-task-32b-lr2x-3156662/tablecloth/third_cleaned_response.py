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
        print("Error creating output directory ")


sys = chrono.ChSystemSMC()  


mesh = fea.ChMesh()  


sys.Add(mesh)



print("error happened with only start ```python")