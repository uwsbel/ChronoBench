import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import errno
import os

# Output directory
out_dir = chrono.GetChronoOutputPath() + "FEA_SHELLS_BST"

# Create (if needed) the output directory
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
        print("Error creating output directory")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

# Create a mesh, a container for groups of elements and their referenced nodes
mesh = fea.ChMesh()
sys.Add(mesh)

# Disable gravity for the system (options)
# mesh.SetAutomaticGravity(False)

# Define nodes to plot or load
nodePlotA = fea.ChNodeFEAxyz()
nodePlotB = fea.ChNodeFEAxyz()
nodesLoad = []

# Create interpolation functions for reference tracking (if needed)
ref_X = chrono.ChFunctionInterp()
ref_Y = chrono.ChFunctionInterp()

# Define load force vector
load_force = chrono.ChVector3d()

# Monitoring nodes and elements
mnodemonitor = fea.ChNodeFEAxyz()
melementmonitor = fea.ChElementShellBST()

if (True):
    # Define material properties
    density = 100
    E = 6e4
    nu = 0.0
    thickness = 0.01

    # Create isotropic Kirchhoff material elasticity object
    melasticity = fea.ChElasticityKirchhoffIsotropic(E, nu)

    # Create material object by assigning the elasticity property
    material = fea.ChMaterialShellKirchhoff(melasticity)
    material.SetDensity(density)

    # Define the mesh dimensions
    L_x = 1
    nsections_x = 40
    L_z = 1
    nsections_z = 40

    # Create list to hold the nodes
    mynodes = []

    # Create nodes for the mesh grid
    for iz in range(nsections_z + 1):
        for ix in range(nsections_x + 1):
            p = chrono.ChVector3d(ix * (L_x / nsections_x), 0, iz * (L_z / nsections_z))
            mnode = fea.ChNodeFEAxyz(p)
            mesh.AddNode(mnode)
            mynodes.append(mnode)

    # Create elements and associate nodes
    for iz in range(nsections_z):
        for ix in range(nsections_x):
            # Create first element
            melementA = fea.ChElementShellBST()
            mesh.AddElement(melementA)

            if (iz == 0 and ix == 1):
                melementmonitor = melementA

            # Define boundary nodes
            boundary_1 = mynodes[(iz + 1) * (nsections_x + 1) + ix + 1]
            boundary_2 = mynodes[(iz + 1) * (nsections_x + 1) + ix - 1] if (ix > 0) else None
            boundary_3 = mynodes[(iz - 1) * (nsections_x + 1) + ix + 1] if (iz > 0) else None

            # Set nodes to the element
            melementA.SetNodes(
                mynodes[(iz) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementA.AddLayer(thickness, 0 * chrono.CH_DEG_TO_RAD, material)

            # Create second element
            melementB = fea.ChElementShellBST()
            mesh.AddElement(melementB)

            # Define boundary nodes
            boundary_1 = mynodes[(iz) * (nsections_x + 1) + ix]
            boundary_2 = mynodes[(iz) * (nsections_x + 1) + ix + 2] if (ix < nsections_x - 1) else None
            boundary_3 = mynodes[(iz + 2) * (nsections_x + 1) + ix] if (iz < nsections_z - 1) else None

            # Set nodes to the element
            melementB.SetNodes(
                mynodes[(iz + 1) * (nsections_x + 1) + ix + 1],
                mynodes[(iz + 1) * (nsections_x + 1) + ix],
                mynodes[(iz) * (nsections_x + 1) + ix + 1],
                boundary_1, boundary_2, boundary_3
            )

            # Add layer to the element
            melementB.AddLayer(thickness, 0 *
,




)
,
,
,
,
,,, ( (,



,
,

,
,
,
,
,
,
,
,
,_
,
,
,


,
,






,
,
,



(
,
,

,






,

,
,,
,
,
,,,



,,
,





,



,

,
,,,
,

,
,
,,,,,,,,
,


,
,
,,
,
,
,
,
,

_



,











,
,
,
(

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
,)
,
,
,
(
, if,
,

)

,
,


,,,,,,,,,,,,,,,,,,
,,,,,
,,,
,
,,,
,,,,,,,
,
,
,
,,,
,,,,,



,




,
,,
)
)
,

,
)



,,,,,

,
,,,
,,,,,,
,,,,





,


)
)
)










_


(
(






_










,
,))
)


(



,
,
,
,
,
,
,
,

,,,,





(

































,
,




,
































,,,



,
,,,




,,







,,


,,,,







,





,

,





















,




,
,
,,

,


)
)






































,













 if















, if if,






,
,













,
,
,,,
,,
,,(,(,,,
,


,









,





,

,



,,,
,,,,,,
, if,
,
,
,

,



 if,
,
,
,,,,
,
,,,, if, if,,,,,,
,
,
, if,,
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
,,,
,
,,,
,
,,,,
,
,
,,,,,,,,,,
,,,,,
,,,,,,,,
,
,
,
,
,,
,
,
,
,
,,,

,
,,,,
,
,
,



,,,,,,
,



,
,,
,,
,,,,



,

,
,,,,




,




,,



,,,





,
,,
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




,,
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
,,










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
)
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










,



,
,,


,


    
    ,




,
,



,,
,


,





    
    



































,,





,





)
,


,



   
,

,
,
,
,
,_
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
,(





,,
,
,,
,
,
,
,



,


,

,




_
,,,
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





,,,,



   
,





,,
,
,






,
,,
,,,
,         ,








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
,,,,,,,,,,,,,,,,,,,

,
,,


,
,
,
,,
,
,
,,
,,,,,,,,,,,,,,


,
,,,,,,,,
,
,,,,,


,,,,,,
,
,
,,,,,,,,,,
,

)
,




)
,








,,
,
,
,
,

















,
,,
,
,



,,,,,,,
,,,
,
,
,
,
,


,,







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




,,,
,
print("error happened with only start ```python")