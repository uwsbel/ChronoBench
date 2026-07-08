"""A CORRECT turn-1 candidate in a different style: corner geometry generated from a spec
table, joints collected in dictionaries, a small Car class mirroring the C++ MySimpleCar,
manual CSV assembly. Same bodies, anchors, spring constants, solver, and drop, so the same
settle bands; must score ~100."""
import json

import pychrono as ch

DT = 5e-3
DURATION = 4.0


class Car:
    def __init__(self, world):
        chassis_mat = ch.ChContactMaterialNSC()
        self.wheel_mat = ch.ChContactMaterialNSC()
        self.wheel_mat.SetFriction(1.0)

        self.body = ch.ChBodyEasyBox(1, 0.5, 3, 1.0, False, True, chassis_mat)
        self.body.SetPos(ch.ChVector3d(0, 1, 0))
        self.body.SetMass(150)
        self.body.SetInertiaXX(ch.ChVector3d(4.8, 4.5, 1))
        world.AddBody(self.body)

        self.springs = []
        self.motors = []
        for sx, sz, is_front in ((1, 1, True), (-1, 1, True), (1, -1, False), (-1, -1, False)):
            self._corner(world, sx, sz, is_front)

    def _corner(self, world, sx, sz, is_front):
        hub = ch.ChBodyEasyBox(0.1, 0.4, 0.4, 1.0, False, False)
        hub.SetPos(ch.ChVector3d(1.3 * sx, 1, sz))
        hub.SetMass(8)
        hub.SetInertiaXX(ch.ChVector3d(0.2, 0.2, 0.2))
        world.AddBody(hub)

        tire = ch.ChBodyEasyCylinder(ch.ChAxis_Y, 0.45, 0.3, 1.0, False, True, self.wheel_mat)
        tire.SetPos(ch.ChVector3d(1.5 * sx, 1, sz))
        tire.SetRot(ch.QuatFromAngleZ(ch.CH_PI_2))
        tire.SetMass(3)
        tire.SetInertiaXX(ch.ChVector3d(0.2, 0.2, 0.2))
        world.AddBody(tire)

        pivot = ch.ChLinkLockRevolute()
        pivot.Initialize(tire, hub, ch.ChFramed(ch.ChVector3d(1.5 * sx, 1, sz), ch.QuatFromAngleY(ch.CH_PI_2)))
        world.AddLink(pivot)

        wishbone_rows = ((1.2, 0.2), (1.2, -0.2), (0.8, 0.2), (0.8, -0.2))
        for ay, dz in wishbone_rows:
            rod = ch.ChLinkDistance()
            rod.Initialize(self.body, hub, False,
                           ch.ChVector3d(0.5 * sx, ay, (1 + dz) * sz),
                           ch.ChVector3d(1.25 * sx, ay, sz))
            world.AddLink(rod)

        coil = ch.ChLinkTSDA()
        coil.Initialize(self.body, hub, False,
                        ch.ChVector3d(0.5 * sx, 1.2, sz),
                        ch.ChVector3d(1.25 * sx, 0.8, sz))
        coil.SetSpringCoefficient(28300.0)
        coil.SetDampingCoefficient(80.0)
        world.AddLink(coil)
        self.springs.append(coil)

        tie = ch.ChLinkDistance()
        tie.Initialize(self.body, hub, False,
                       ch.ChVector3d(0.5 * sx, 1.21, 1.4 * sz),
                       ch.ChVector3d(1.25 * sx, 1.21, 1.3 * sz))
        world.AddLink(tie)

        if not is_front:
            drive = ch.ChLinkMotorRotationTorque()
            drive.Initialize(tire, self.body,
                             ch.ChFramed(ch.ChVector3d(1.5 * sx, 1, sz), ch.QuatFromAngleY(ch.CH_PI_2)))
            world.AddLink(drive)
            self.motors.append(drive)

    def update_drivetrain(self, throttle):
        shaft = 5.0 * 0.5 * (self.motors[0].GetMotorAngleDt() + self.motors[1].GetMotorAngleDt())
        engine = shaft / 0.3
        torque = (80.0 - engine * 0.1) * throttle
        per_wheel = 0.5 * (torque / 0.3) / 0.2
        for m in self.motors:
            m.SetTorqueFunction(ch.ChFunctionConst(per_wheel))


world = ch.ChSystemNSC()
world.SetGravityY()
world.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)

ground_mat = ch.ChContactMaterialNSC()
ground_mat.SetStaticFriction(1.0)
ground_mat.SetSlidingFriction(1.0)
floor = ch.ChBodyEasyBox(60, 2, 60, 1.0, False, True, ground_mat)
floor.SetPos(ch.ChVector3d(0, -1, 0))
floor.SetFixed(True)
world.AddBody(floor)

car = Car(world)

world.SetSolverType(ch.ChSolver.Type_PSOR)
world.GetSolver().AsIterative().SetMaxIterations(20)

lines = ["t,cy,slen,cz"]
history = []
t = 0.0
while t < DURATION:
    t = world.GetChTime()
    car.update_drivetrain(0.0)
    world.DoStepDynamics(DT)
    p = car.body.GetPos()
    rec = (t, p.y, car.springs[0].GetLength(), p.z)
    history.append(rec)
    lines.append(f"{rec[0]:.6f},{rec[1]:.6e},{rec[2]:.6e},{rec[3]:.6e}")

with open("out.csv", "w") as fh:
    fh.write("\n".join(lines) + "\n")

late = [rec for rec in history if rec[0] >= DURATION - 1.0]
print(json.dumps({"chassis_y_settled": sum(rec[1] for rec in late) / len(late),
                  "spring_len_settled": sum(rec[2] for rec in late) / len(late),
                  "throttle": 0.0}))
