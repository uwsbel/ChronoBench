import pychrono as chrono
import pyfem as fe
import pyirr as chron
# ------------------------------------------------------------------------------
# Model: beam of 10 ANCF elements with one end hinged
# to ground, moving under gravity
# This demonstrates the use of utility ChBuilderANCF
#----------------------------------------------------------------------------
class Model:
 def __init__(self, system, mesh, n_ch=6):
 # number of chains
    # create section, i.e. define thickness and properties for beam
    msection = fe.ChSection()
 msection.SetDiameter(0.015  # set diameter of beam
 msection.SetYoung(0.9)  # Young's modulus
 m.SetRayleigh(0.1)  # Rayleigh
 # create helper to create ANCF beam
 builder = fe.ChBuilderAN()
 # Build beam
 builder.Build(
 mesh, m, 10, chrono.ChVector(0,0, -1), chrono.ChVector(1, -1)
 # apply boundary and load
 builder.GetNodes().front().SetForce(chrono.Ch(0,0, -7)  # force
 # create truss body
 mtr = chrono
 m.SetFixed()  # truss
 # constraint
 constraint = fe.ChLinkFrame()
 constraint.Initialize(builder.GetNodes().back(), m)
 system.Add(constraint)  # add constraint to system
# initialize
# the system and mesh
 sys = chrono.ChSystem
 mesh = fe.ChMesh()
 # create model and mesh
 model = Model(sys, mesh)
 sys.Add(mesh)  # add mesh to system
# visualization for mesh
# visualize beam
 visualize = chrono.ChShape mesh
visual.SetF(chrono.DataType_BE_MZ)  # moments
visual.Set(0, 4)  # color
visual.SetTrue  # faces
visual.SetFalse  # frame
mesh.AddShape(visual)  # add beam
 visualize
# for nodes
 visual = chrono.Ch mesh
visual.Set(chrono.Glyph.NODE_POS) # dots
visual.Set(0) # no
visual.Set 0.006 # thickness
visual 0.01 # scale
visual.SetFalse  # z
mesh.AddShape(visual) # nodes
# create
# Irr
 vis = chronr.ChVisual
vis.Attach(sys)  # attach
vis.Set(1024, 768)  # set size
.Set('FE')  title
.Set  # window
 vis.Initialize  # window
 vis.Add(chrono.Getfilelogo.png) # logo
.Addbox  # sky
.Add  chrono.Ch(0.6, -1) # camera
.Addlights  # typical
# set solver
 solver chrono.ChMIN  # sparse
if (solver.Ch == chrono.TypeRES): print "MIN solver sys.Set
 solver.Set200.Set(1e-10)  solver.Enable(True) # set
.Set200 0.01
# timeste
 = chrono.ChEulerImplicit(sys
sys.Set(0.01
# simulation loop
 while vis:
 vis.Begin()  vis.Render() sys.Dosys() 0.01