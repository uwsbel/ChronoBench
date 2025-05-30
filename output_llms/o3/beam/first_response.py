#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Beam FEA demo – PyChrono
Author : <your-name>
Date   : <today>

This example shows how to:
 1. Create a ChSystem (SMC) and set global parameters.
 2. Build an FEA mesh made of Euler-Bernoulli beams.
 3. Add constraints, loads, and visualization assets.
 4. Use the Irrlicht real-time renderer (ChIrrApp) to run the loop.
"""

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math
import sys

# 1. -------------------------------------------------------------------------
# INITIALIZE CHRONO
chrono.SetChronoDataPath(chrono.GetChronoDataPath())   # use default data folder

system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))        # gravity

# 2. -------------------------------------------------------------------------
# CREATE AN FEA MESH WITH BEAMS

mesh = chrono.ChMesh()

# 2.1  Beam section (material + geometric properties) ------------------------
section = chrono.ChBeamSectionAdvanced()

E  = 2.1e11             # Young modulus      [Pa]
G  = 8.0e10             # Shear modulus      [Pa]
rho= 7800               # Density            [kg/m^3]
d  = 0.03               # Circular beam dia. [m]

section.SetAsCircularSection(d)          # geometric set-up (A, Ixx, Iyy, etc.)
section.SetYoungModulus(E)
section.SetGshearModulus(G)
section.SetDensity(rho)
section.SetBeamRaleyghDamping(0.0002)    # optional

# 2.2  Build a straight cantilever made of N_SEG finite elements -------------
N_SEG  = 10                  # number of beam elements
L      = 1.0                 # total length [m]
dx     = L / N_SEG           # element size

prev_node = None

for i in range(N_SEG+1):
    # Position of the node along the global X axis
    pos = chrono.ChVectorD(i * dx, 0, 0)
    
    # Create a beam node with 6 DOFs (= position + rotation)
    node = chrono.ChNodeFEAxyzrot(chrono.ChFrameD(pos))
    node.SetMass(0)          # mass comes from beam elements
    mesh.AddNode(node)
    
    # First node will be fixed to ground later  (cantilever)
    if prev_node:
        # Create the beam element between prev_node and node
        element = chrono.ChElementBeamEuler()
        element.SetNodes(prev_node, node)
        element.SetSection(section)
        mesh.AddElement(element)
    
    prev_node = node

# 2.3  Fix first node to ground ----------------------------------------------
ground = chrono.ChBody()           # empty body with zero mass
ground.SetBodyFixed(True)
system.Add(ground)

link = chrono.ChLinkMateGeneric()
link.Initialize(ground, mesh.GetNode(0), False,
                mesh.GetNode(0).Frame(), mesh.GetNode(0).Frame())
system.Add(link)

# 2.4  Add the mesh to the physical system -----------------------------------
system.Add(mesh)

# 3. -------------------------------------------------------------------------
# VISUALIZATION ASSETS

# Smooth color plot of Von-Mises stress
vis_beam = chrono.ChVisualizationFEAmesh(mesh)
vis_beam.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_CHR_STRESS_VONMISES)
vis_beam.SetColorscaleMinMax(0, 1.0e7)
vis_beam.SetSmoothFaces(True)
vis_beam.SetWireframe(False)
mesh.AddAsset(vis_beam)

# Draw undeformed wireframe for reference
vis_wire = chrono.ChVisualizationFEAmesh(mesh)
vis_wire.SetFEMdataType(chrono.ChVisualizationFEAmesh.E_PLOT_NONE)
vis_wire.SetWireframe(True)
vis_wire.SetWireframeThickness(0.002)
mesh.AddAsset(vis_wire)

# Optional: show nodes as small spheres
vis_nodes = chrono.ChVisualizationFEAmesh(mesh)
vis_nodes.SetFEMglyphType(chrono.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
vis_nodes.SetSymbolsThickness(0.006)
mesh.AddAsset(vis_nodes)

# 4. -------------------------------------------------------------------------
# SETUP IRRLICHT APPLICATION

app = chronoirr.ChIrrApp(system, "Beam FEA – PyChrono", chronoirr.dimension2du(1024,768))
app.AddTypicalCamera(chronoirr.vector3df(0.5, 0.3, 0.7), chronoirr.vector3df(0,0,0))  # cam position, target
app.AddTypicalLights()
app.AddSkyBox()
app.AssetBindAll()
app.AssetUpdateAll()

# 5. -------------------------------------------------------------------------
# APPLY A TIP LOAD  (simple constant force) ----------------------------------

tip_node = mesh.GetNode(N_SEG)               # last node
load_container = chrono.ChLoadContainer()
system.Add(load_container)

# Create a custom follower load for the tip node -----------------------------
class TipLoad(chrono.ChLoaderU):
    def __init__(self, loadable):
        chrono.ChLoaderU.__init__(self, loadable)  # must call base constructor
        
    # The load is constant, so no dependency on coordinates U (here length param)
    def ComputeF(self, U, V, W):
        return chrono.ChVectorD(0, -200, 0)  # 200 N downward

# Use a ChLoad for a single node (follower = force sticks to node orientation)
load = chrono.ChLoad< chrono.ChLoaderPoint >(
    tip_node, TipLoad(tip_node))
load_container.Add(load)

# 6. -------------------------------------------------------------------------
# SIMULATION LOOP

time_step = 1e-3

while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()
    
    system.DoStepDynamics(time_step)