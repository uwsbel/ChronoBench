"""YAML-declared slider-crank, turn 1 (CREATE) -- PyChrono 10.0, headless -- contracted reference.

The model is DECLARED in Chrono's MBS-YAML schema: this script writes the model, simulation, and
solver YAML files, then loads them with chrono::parsers::ChParserMbsYAML and runs headless.
Mechanism: crank pivot at the origin (motor-driven revolute about +Y at a constant omega = pi
rad/s), crank pin at radius r = 0.25 m, connecting rod of length l = 1.0 m realized as a DISTANCE
constraint (attachment points in GLOBAL coordinates of the initial assembly), slider on a
prismatic guide along +X. Inline slider-crank identity: stroke = 2 r = 0.5 m (independent oracle).

Schema notes baked into this reference: the parser constructor takes the SIMULATION yaml (absolute
path is safest); the simulation yaml references the model and solver files by bare filename
(resolved against its own directory); DISTANCE constraint point1/point2 are world coordinates.
"""
import csv
import json
import math
import os

import pychrono as chrono
import pychrono.parsers as parsers

r, l, omega = 0.25, 1.0, math.pi
t_end, dt = 4.0, 1.0e-3

MODEL = f"""
chrono-version: 10.0
model:
  name: yaml_slider_crank
  bodies:
    - name: ground
      location: [0, 0, 0]
      fixed: true
    - name: crank
      location: [0, 0, 0]
      mass: 1.0
      com:
        location: [0, 0, 0]
        orientation: [0, 0, 0]
      inertia:
        moments: [0.01, 0.01, 0.01]
        products: [0, 0, 0]
    - name: slider
      location: [{r + l}, 0, 0]
      mass: 1.0
      com:
        location: [0, 0, 0]
        orientation: [0, 0, 0]
      inertia:
        moments: [0.02, 0.02, 0.02]
        products: [0, 0, 0]
  joints:
    - name: ground_slider
      type: PRISMATIC
      body1: ground
      body2: slider
      location: [{r + l}, 0, 0]
      axis: [1, 0, 0]
  constraints:
    - name: crank_slider
      type: DISTANCE
      body1: crank
      body2: slider
      point1: [{r}, 0, 0]
      point2: [{r + l}, 0, 0]
  motors:
    - name: drive
      type: ROTATION
      spindle: REVOLUTE
      body1: ground
      body2: crank
      location: [0, 0, 0]
      axis: [0, 1, 0]
      actuation_type: SPEED
      actuation_function:
        type: CONSTANT
        value: {omega}
"""

SIM = f"""
chrono-version: 10.0
type: MBS
model: "model_sc.yaml"
solver: "solver_sc.yaml"
simulation:
  end_time: {t_end}
  enforce_realtime: false
"""

SOLVER = """
chrono-version: 10.0
contact_method: SMC
integrator:
  type: Euler_implicit_linearized
  time_step: 1e-3
solver:
  type: Barzilai_Borwein
  max_iterations: 100
"""

open("model_sc.yaml", "w").write(MODEL)
open("sim_sc.yaml", "w").write(SIM)
open("solver_sc.yaml", "w").write(SOLVER)

parser = parsers.ChParserMbsYAML(os.path.abspath("sim_sc.yaml"), False)
sys = parser.CreateSystem()
parser.Populate(sys)
slider = sys.SearchBody("slider")

ts, xs = [], []
t = 0.0
while t < t_end:
    sys.DoStepDynamics(dt)
    t += dt
    ts.append(t)
    xs.append(slider.GetPos().x)

with open("out.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t", "slider_x"])
    for tt, x in zip(ts, xs):
        w.writerow([f"{tt:.6f}", f"{x:.6e}"])

print(json.dumps({"stroke": max(xs) - min(xs), "r": r}))
