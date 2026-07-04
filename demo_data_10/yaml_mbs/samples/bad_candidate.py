"""A RUNS-BUT-WRONG turn-1 YAML slider-crank: executes cleanly and passes the minimal L2 text
checks, but the DISTANCE constraint's point2 is declared at the PIVOT ([0,0,0]) instead of at the
slider ([r+l,0,0]), the exact schema misreading a model is likely to make (the MBS-YAML constraint
points are GLOBAL assembly coordinates, not body-local). The crank pin stays at constant distance
from the pivot, so the constraint is trivially satisfied and the slider NEVER MOVES: stroke 0
instead of 0.5. The CSV-derived stroke invariant catches it and the wrong-physics cap applies."""
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
      point2: [0, 0, 0]
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
