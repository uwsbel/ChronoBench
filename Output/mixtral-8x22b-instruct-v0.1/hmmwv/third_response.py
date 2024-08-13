if (eff_time < 2):
    self.SetSteering(0.0)
else:
    self.SetSteering(0.3 * m.sin(2.0 * m.pi * (eff_time - 2) / 6))