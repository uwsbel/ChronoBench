###############################################################
# PyChrono demo : ANCF cable-beam, one end hinged to ground   #
#                                                             #
# The model is a straight, slender beam discretised with      #
# ANCF cable elements.  The first node is fixed (“hinged”)    #
# to an absolute-reference body (the ground). Gravity acts    #
# in –Y and the beam deformation is rendered with the         #
# Irrlicht visualiser.                                        #
###############################################################

# ----------------------------------------------------------------
# 1.  Imports & basic Chrono initialisation
# ----------------------------------------------------------------
import math
import pychrono.core      as chrono
import pychrono.irrlicht  as chronoirr
import pychrono.fea       as fea            #  << FEA module

# Global Chrono settings
chrono.SetChronoDataPath( chrono.GetChronoDataPath() )    # texture / mesh path
sys = chrono.ChSystemSMC()                                # SMC contact not really used
sys.Set_G_acc( chrono.ChVectorD(0, -9.81, 0) )            # gravity


# ----------------------------------------------------------------
# 2.  FEA mesh – ANCF cable elements
# ----------------------------------------------------------------
mesh          = fea.ChMesh()
sys.Add(mesh)

# ------- material
density       = 7.8e3                    # [kg/m^3]  (steel)
E             = 2.1e11                   # [Pa]
G             = 8.1e10                   # [Pa] (not strictly required for cables)
diam          = 0.02                     # [m]
A             = math.pi * (diam**2) / 4  # cross-sectional area
I             = math.pi * (diam**4) / 64 # second moment of area
mat_cable     = fea.ChMaterialCableANCF(E, G, density)

# ------- geometry discretisation
L             = 2.0          # total beam length  [m]
Ne            = 20           # number of cable elements
dx            = L / Ne

# ------- create nodes
nodes = []
for i in range(Ne + 1):
    pos = chrono.ChVectorD(i * dx, 0.0, 0.0)
    n   = fea.ChNodeFEAxyzD(pos)   # xyz displacement + director
    n.SetMass(density * A * dx)    # lump mass roughly
    mesh.AddNode(n)
    nodes.append(n)

# fix (hinge) the first node to ground
nodes[0].SetFixed(True)


# ------- create ANCF cable elements
for i in range(Ne):
    cab = fea.ChElementCableANCF()
    cab.SetNodes( nodes[i], nodes[i+1] )
    cab.SetMaterial(mat_cable)
    cab.SetSectionRadius(diam/2)
    cab.SetAlphaDamp(0.02)          # small Rayleigh damping
    mesh.AddElement(cab)


# ----------------------------------------------------------------
# 3.  FEA visualisation assets
# ----------------------------------------------------------------
#  – glyphs for nodes
mesh_vis_nodes = fea.ChVisualizationFEAmesh(mesh)
mesh_vis_nodes.SetFEMglyphType( fea.ChVisualizationFEAmesh.GlyphType_NODE_DOT_POS )
mesh_vis_nodes.SetSymbolsThickness(0.008)
mesh_vis_nodes.SetDataType( fea.ChVisualizationFEAmesh.DataType_NONE )
mesh.AddAsset(mesh_vis_nodes)

#  – beam surface (cylinders along elements)
mesh_vis_cable = fea.ChVisualizationFEAmesh(mesh)
mesh_vis_cable.SetFEMdatasmooth(True)
mesh_vis_cable.SetWireframe(False)
mesh_vis_cable.SetDataType( fea.ChVisualizationFEAmesh.DataType_NONE )
mesh.AddAsset(mesh_vis_cable)


# ----------------------------------------------------------------
# 4.  Irrlicht application
# ----------------------------------------------------------------
app = chronoirr.ChIrrApp(sys,
                         "ANCF Cable beam – hinged end",
                         chronoirr.dimension2du(1024, 768))

app.AddTypicalSky()
app.AddTypicalLogo()
app.AddTypicalLights()
app.AddTypicalCamera( chronoirr.vector3df(-1.6, 0.6, 1.6),
                      chronoirr.vector3df( 0.8, 0.0, 0.0) )

# bind & update assets once before the loop
app.AssetBindAll()
app.AssetUpdateAll()


# ----------------------------------------------------------------
# 5.  Time stepping loop
# ----------------------------------------------------------------
h          = 1e-3    # step size
t_end      = 5       # [s]

while(app.GetDevice().run()):
    if sys.GetChTime() >= t_end:
        break

    app.BeginScene()
    app.DrawAll()
    app.EndScene()

    sys.DoStepDynamics(h)

print("Simulation finished at t =", sys.GetChTime(), "s")