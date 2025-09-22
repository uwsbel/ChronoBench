import pychrono as chrono
import pychrono.irrlicht as irr
import math as m
import random as rd
import numpy as np
import os



def main():
    

    
    
    

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowTitle(title)
    vis.SetWindowSize(screen_width, screen_height)
    vis.SetChaseCamera(track_body, 5.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddTypicalLights()

    
    cam = vis.GetCamera()
    cam.SetAspectRatio(screen_width / screen_height)
    cam.SetPosition(track_body.GetPos())

    
    solver = chrono.ChConcaSolverSMC()
    sys.GetSystem().SetSolver(solver)

    
    
    

    
    nparticles = 1000

    
    particles = []

    
    bodies = []

    
    shapes = []

    
    trimesh_shapes = []

    
    trimesh = []

    
    trimesh2 = []

    
    trimesh3 = []

    
    trimesh4 = []

    
    trimesh5 = []

    
    trimesh6 = []

    
    trimesh7 = []

    
    trimesh8 = []

    
    trimesh9 = []

    
    trimesh10 = []

    
    trimesh11 = []

    
    trimesh12 = []

    
    trimesh13 = []

    
    trimesh14 = []

    
    trimesh15 = []

    
    trimesh16 = []

    
    trimesh17 = []

    
    trimesh18 = []

    
    trimesh19 = []

    
    trimesh20 = []

    
    trimesh21 = []

    
    trimesh22 = []

    
    trimesh23 = []

    
    trimesh24 = []

    
    trimesh25 = []

    
    trimesh26 = []

    
    trimesh27 = []

    
    trimesh28 = []

    
    trimesh29 = []

    
    trimesh30 = []

    
    trimesh31 = []

    
    trimesh32 = []

    
    trimesh33 = []

    
    trimesh34 = []

    
    trimesh35 = []

    
    trimesh36 = []

    
    trimesh37 = []

    
    trimesh38 = []

    
    trimesh39 = []

    
    trimesh40 = []

    
    trimesh41 = []

    
    trimesh42 = []

    
    trimesh43 = []

    
    trimesh44 = []

    
    trimesh45 = []

    
    trimesh46 = []

    
    trimesh47 = []

    
    trimesh48 = []

    
    trimesh49 = []

    
    trimesh50 = []

    
    trimesh51 = []

    
    trimesh52 = []

    
    trimesh53 = []

    
    trimesh54 = []

    
    trimesh55 = []

    
    trimesh56 = []

    
    trimesh57 = []

    
    trimesh58 = []

    
    trimesh59 = []

    
    trimesh60 = []

    
    trimesh61 = []

    
    trimesh62 = []

    
    trimesh63 = []

    
    trimesh64 = []

    
    trimesh65 = []

    
    trimesh66 = []

    
    trimesh67 = []

    
    trimesh68 = []

    
    trimesh69 = []

    
    trimesh70 = []

    
    trimesh71 = []

    
    trimesh72 = []

    
    trimesh73 = []

    
    trimesh74 = []

    
    trimesh75 = []

    
    trimesh76 = []

    
    trimesh77 = []

    
    trimesh78 = []

    
    trimesh79 = []

    
    trimesh80 = []

    
    trimesh81 = []

    
    trimesh82 = []

    
    trimesh83 = []

    
    trimesh84 = []

    
    trimesh85 = []

    
    trimesh86 = []

    
    trimesh87 = []

    
    trimesh88 = []

    
    trimesh89 = []

    
    trimesh90 = []

    
    trimesh91 = []

    
    trimesh92 = []

    
    trimesh93 = []

    
    trimesh94 = []

    
    trimesh95 = []

    
    trimesh96 = []

    
    trimesh97 = []

    
    trimesh98 = []

    
    trimesh99 = []

    
    trimesh100 = []

    
    trimesh101 = []

    
    trimesh102 = []

    
    trimesh103 = []

    
    trimesh104 = []

    
    trimesh105 = []

    
    trimesh106 = []

    
    trimesh107 = []

    
    trimesh108 = []

    
    trimesh109 = []

    
    trimesh110 = []

    
    trimesh111 = []

    
    trimesh112 = []

    
    trimesh113 = []

    
    trimesh114 = []

    
    trimesh115 = []

    
    trimesh116 = []

    
    trimesh117 = []

    
    trimesh118 = []

    
    trimesh119 = []

    
    trimesh120 = []

    
    trimesh121 = []

    
    trimesh122 = []

    
    trimesh123 = []

    
    trimesh124 = []

    
    trimesh125 = []

    
    trimesh126 = []

    
    trimesh127 = []

    
    trimesh128 = []

    
    trimesh129 = []

    
    trimesh130 = []

    
    trimesh131 = []

    
    trimesh132 = []

    
    trimesh133 = []

    
    trimesh134 = []

    
    trimesh135 = []

    
    trimesh136 = []

    
    trimesh137 = []

    
    trimesh138 = []

    
    trimesh139 = []

    
    trimesh140 = []

    
    trimesh141 = []

    
    trimesh142 = []

    
    trimesh143 = []

    
    trimesh144 = []

    
    trimesh145 = []

    
    trimesh146 = []

    
    trimesh147 = []

    
    trimesh148 = []

    
    trimesh149 = []

    
    trimesh150 = []

    
    trimesh151 = []

    
    trimesh152 = []

    
    trimesh153 = []

    
    trimesh154 = []

    
    trimesh155 = []

    
    trimesh156 = []

    
    trimesh157 = []

    
    trimesh158 = []

    
    trimesh159 = []

    
    trimesh160 = []

    
    trimesh161 = []

    
    trimesh162 = []

    
    trimesh163 = []

    
    trimesh164 = []

    
    trimesh165 = []

    
    trimesh166 = []

    
    trimesh167 = []

    
    trimesh168 = []

    
    trimesh169 = []

    
    trimesh170 = []

    
    trimesh171 = []

    
    trimesh172 = []

    
    trimesh173 = []

    
    trimesh174 = []

    
    trimesh175 = []

    
    trimesh176 = []

    
    trimesh177 = []

    
    trimesh178 = []

    
    trimesh179 = []

    
    trimesh180 = []

    
    trimesh181 = []

    
    trimesh182 = []

    
    trimesh183 = []

    
    trimesh184 = []

    
    trimesh185 = []

    
    trimesh186 = []

    
    trimesh187 = []

    
    trimesh188 = []

    
    trimesh189 = []

    
    trimesh190 = []

    
    trimesh191 = []