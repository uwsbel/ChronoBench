import pychrono as chrono
import pychrono.irrlicht as irr
import math as m
import random as rd
import numpy as np
import os

# =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the various modules
    # --------------------------

    # Create the Irrlicht visualization
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowTitle(title)
    vis.SetWindowSize(screen_width, screen_height)
    vis.SetChaseCamera(track_body, 5.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()

    # Create the irrlicht camera controller
    cam = vis.GetCamera()
    cam.SetAspectRatio(screen_width / screen_height)
    cam.SetPosition(track_body.GetPos())

    # Create the solver
    solver = chrono.ChConcaSolverSMC()
    sys.GetSystem().SetSolver(solver)

    # --------------------------
    # Simulation loop
    # --------------------------

    # Number of particles to generate
    nparticles = 1000

    # Create a list to store the particles
    particles = []

    # Create a list to store the rigid bodies
    bodies = []

    # Create a list to store the shapes
    shapes = []

    # Create a list to store the trimesh shapes
    trimesh_shapes = []

    # Create a list to store the trimesh
    trimesh = []

    # Create a list to store the trimesh
    trimesh2 = []

    # Create a list to store the trimesh
    trimesh3 = []

    # Create a list to store the trimesh
    trimesh4 = []

    # Create a list to store the trimesh
    trimesh5 = []

    # Create a list to store the trimesh
    trimesh6 = []

    # Create a list to store the trimesh
    trimesh7 = []

    # Create a list to store the trimesh
    trimesh8 = []

    # Create a list to store the trimesh
    trimesh9 = []

    # Create a list to store the trimesh
    trimesh10 = []

    # Create a list to store the trimesh
    trimesh11 = []

    # Create a list to store the trimesh
    trimesh12 = []

    # Create a list to store the trimesh
    trimesh13 = []

    # Create a list to store the trimesh
    trimesh14 = []

    # Create a list to store the trimesh
    trimesh15 = []

    # Create a list to store the trimesh
    trimesh16 = []

    # Create a list to store the trimesh
    trimesh17 = []

    # Create a list to store the trimesh
    trimesh18 = []

    # Create a list to store the trimesh
    trimesh19 = []

    # Create a list to store the trimesh
    trimesh20 = []

    # Create a list to store the trimesh
    trimesh21 = []

    # Create a list to store the trimesh
    trimesh22 = []

    # Create a list to store the trimesh
    trimesh23 = []

    # Create a list to store the trimesh
    trimesh24 = []

    # Create a list to store the trimesh
    trimesh25 = []

    # Create a list to store the trimesh
    trimesh26 = []

    # Create a list to store the trimesh
    trimesh27 = []

    # Create a list to store the trimesh
    trimesh28 = []

    # Create a list to store the trimesh
    trimesh29 = []

    # Create a list to store the trimesh
    trimesh30 = []

    # Create a list to store the trimesh
    trimesh31 = []

    # Create a list to store the trimesh
    trimesh32 = []

    # Create a list to store the trimesh
    trimesh33 = []

    # Create a list to store the trimesh
    trimesh34 = []

    # Create a list to store the trimesh
    trimesh35 = []

    # Create a list to store the trimesh
    trimesh36 = []

    # Create a list to store the trimesh
    trimesh37 = []

    # Create a list to store the trimesh
    trimesh38 = []

    # Create a list to store the trimesh
    trimesh39 = []

    # Create a list to store the trimesh
    trimesh40 = []

    # Create a list to store the trimesh
    trimesh41 = []

    # Create a list to store the trimesh
    trimesh42 = []

    # Create a list to store the trimesh
    trimesh43 = []

    # Create a list to store the trimesh
    trimesh44 = []

    # Create a list to store the trimesh
    trimesh45 = []

    # Create a list to store the trimesh
    trimesh46 = []

    # Create a list to store the trimesh
    trimesh47 = []

    # Create a list to store the trimesh
    trimesh48 = []

    # Create a list to store the trimesh
    trimesh49 = []

    # Create a list to store the trimesh
    trimesh50 = []

    # Create a list to store the trimesh
    trimesh51 = []

    # Create a list to store the trimesh
    trimesh52 = []

    # Create a list to store the trimesh
    trimesh53 = []

    # Create a list to store the trimesh
    trimesh54 = []

    # Create a list to store the trimesh
    trimesh55 = []

    # Create a list to store the trimesh
    trimesh56 = []

    # Create a list to store the trimesh
    trimesh57 = []

    # Create a list to store the trimesh
    trimesh58 = []

    # Create a list to store the trimesh
    trimesh59 = []

    # Create a list to store the trimesh
    trimesh60 = []

    # Create a list to store the trimesh
    trimesh61 = []

    # Create a list to store the trimesh
    trimesh62 = []

    # Create a list to store the trimesh
    trimesh63 = []

    # Create a list to store the trimesh
    trimesh64 = []

    # Create a list to store the trimesh
    trimesh65 = []

    # Create a list to store the trimesh
    trimesh66 = []

    # Create a list to store the trimesh
    trimesh67 = []

    # Create a list to store the trimesh
    trimesh68 = []

    # Create a list to store the trimesh
    trimesh69 = []

    # Create a list to store the trimesh
    trimesh70 = []

    # Create a list to store the trimesh
    trimesh71 = []

    # Create a list to store the trimesh
    trimesh72 = []

    # Create a list to store the trimesh
    trimesh73 = []

    # Create a list to store the trimesh
    trimesh74 = []

    # Create a list to store the trimesh
    trimesh75 = []

    # Create a list to store the trimesh
    trimesh76 = []

    # Create a list to store the trimesh
    trimesh77 = []

    # Create a list to store the trimesh
    trimesh78 = []

    # Create a list to store the trimesh
    trimesh79 = []

    # Create a list to store the trimesh
    trimesh80 = []

    # Create a list to store the trimesh
    trimesh81 = []

    # Create a list to store the trimesh
    trimesh82 = []

    # Create a list to store the trimesh
    trimesh83 = []

    # Create a list to store the trimesh
    trimesh84 = []

    # Create a list to store the trimesh
    trimesh85 = []

    # Create a list to store the trimesh
    trimesh86 = []

    # Create a list to store the trimesh
    trimesh87 = []

    # Create a list to store the trimesh
    trimesh88 = []

    # Create a list to store the trimesh
    trimesh89 = []

    # Create a list to store the trimesh
    trimesh90 = []

    # Create a list to store the trimesh
    trimesh91 = []

    # Create a list to store the trimesh
    trimesh92 = []

    # Create a list to store the trimesh
    trimesh93 = []

    # Create a list to store the trimesh
    trimesh94 = []

    # Create a list to store the trimesh
    trimesh95 = []

    # Create a list to store the trimesh
    trimesh96 = []

    # Create a list to store the trimesh
    trimesh97 = []

    # Create a list to store the trimesh
    trimesh98 = []

    # Create a list to store the trimesh
    trimesh99 = []

    # Create a list to store the trimesh
    trimesh100 = []

    # Create a list to store the trimesh
    trimesh101 = []

    # Create a list to store the trimesh
    trimesh102 = []

    # Create a list to store the trimesh
    trimesh103 = []

    # Create a list to store the trimesh
    trimesh104 = []

    # Create a list to store the trimesh
    trimesh105 = []

    # Create a list to store the trimesh
    trimesh106 = []

    # Create a list to store the trimesh
    trimesh107 = []

    # Create a list to store the trimesh
    trimesh108 = []

    # Create a list to store the trimesh
    trimesh109 = []

    # Create a list to store the trimesh
    trimesh110 = []

    # Create a list to store the trimesh
    trimesh111 = []

    # Create a list to store the trimesh
    trimesh112 = []

    # Create a list to store the trimesh
    trimesh113 = []

    # Create a list to store the trimesh
    trimesh114 = []

    # Create a list to store the trimesh
    trimesh115 = []

    # Create a list to store the trimesh
    trimesh116 = []

    # Create a list to store the trimesh
    trimesh117 = []

    # Create a list to store the trimesh
    trimesh118 = []

    # Create a list to store the trimesh
    trimesh119 = []

    # Create a list to store the trimesh
    trimesh120 = []

    # Create a list to store the trimesh
    trimesh121 = []

    # Create a list to store the trimesh
    trimesh122 = []

    # Create a list to store the trimesh
    trimesh123 = []

    # Create a list to store the trimesh
    trimesh124 = []

    # Create a list to store the trimesh
    trimesh125 = []

    # Create a list to store the trimesh
    trimesh126 = []

    # Create a list to store the trimesh
    trimesh127 = []

    # Create a list to store the trimesh
    trimesh128 = []

    # Create a list to store the trimesh
    trimesh129 = []

    # Create a list to store the trimesh
    trimesh130 = []

    # Create a list to store the trimesh
    trimesh131 = []

    # Create a list to store the trimesh
    trimesh132 = []

    # Create a list to store the trimesh
    trimesh133 = []

    # Create a list to store the trimesh
    trimesh134 = []

    # Create a list to store the trimesh
    trimesh135 = []

    # Create a list to store the trimesh
    trimesh136 = []

    # Create a list to store the trimesh
    trimesh137 = []

    # Create a list to store the trimesh
    trimesh138 = []

    # Create a list to store the trimesh
    trimesh139 = []

    # Create a list to store the trimesh
    trimesh140 = []

    # Create a list to store the trimesh
    trimesh141 = []

    # Create a list to store the trimesh
    trimesh142 = []

    # Create a list to store the trimesh
    trimesh143 = []

    # Create a list to store the trimesh
    trimesh144 = []

    # Create a list to store the trimesh
    trimesh145 = []

    # Create a list to store the trimesh
    trimesh146 = []

    # Create a list to store the trimesh
    trimesh147 = []

    # Create a list to store the trimesh
    trimesh148 = []

    # Create a list to store the trimesh
    trimesh149 = []

    # Create a list to store the trimesh
    trimesh150 = []

    # Create a list to store the trimesh
    trimesh151 = []

    # Create a list to store the trimesh
    trimesh152 = []

    # Create a list to store the trimesh
    trimesh153 = []

    # Create a list to store the trimesh
    trimesh154 = []

    # Create a list to store the trimesh
    trimesh155 = []

    # Create a list to store the trimesh
    trimesh156 = []

    # Create a list to store the trimesh
    trimesh157 = []

    # Create a list to store the trimesh
    trimesh158 = []

    # Create a list to store the trimesh
    trimesh159 = []

    # Create a list to store the trimesh
    trimesh160 = []

    # Create a list to store the trimesh
    trimesh161 = []

    # Create a list to store the trimesh
    trimesh162 = []

    # Create a list to store the trimesh
    trimesh163 = []

    # Create a list to store the trimesh
    trimesh164 = []

    # Create a list to store the trimesh
    trimesh165 = []

    # Create a list to store the trimesh
    trimesh166 = []

    # Create a list to store the trimesh
    trimesh167 = []

    # Create a list to store the trimesh
    trimesh168 = []

    # Create a list to store the trimesh
    trimesh169 = []

    # Create a list to store the trimesh
    trimesh170 = []

    # Create a list to store the trimesh
    trimesh171 = []

    # Create a list to store the trimesh
    trimesh172 = []

    # Create a list to store the trimesh
    trimesh173 = []

    # Create a list to store the trimesh
    trimesh174 = []

    # Create a list to store the trimesh
    trimesh175 = []

    # Create a list to store the trimesh
    trimesh176 = []

    # Create a list to store the trimesh
    trimesh177 = []

    # Create a list to store the trimesh
    trimesh178 = []

    # Create a list to store the trimesh
    trimesh179 = []

    # Create a list to store the trimesh
    trimesh180 = []

    # Create a list to store the trimesh
    trimesh181 = []

    # Create a list to store the trimesh
    trimesh182 = []

    # Create a list to store the trimesh
    trimesh183 = []

    # Create a list to store the trimesh
    trimesh184 = []

    # Create a list to store the trimesh
    trimesh185 = []

    # Create a list to store the trimesh
    trimesh186 = []

    # Create a list to store the trimesh
    trimesh187 = []

    # Create a list to store the trimesh
    trimesh188 = []

    # Create a list to store the trimesh
    trimesh189 = []

    # Create a list to store the trimesh
    trimesh190 = []

    # Create a list to store the trimesh
    trimesh191 = []

    # Create a list to store the trimes