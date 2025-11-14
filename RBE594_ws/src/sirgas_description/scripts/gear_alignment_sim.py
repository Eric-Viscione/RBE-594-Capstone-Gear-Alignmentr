#!/usr/bin/env python3
import math, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from collections import deque

def spectrum_amp(x, fs, freq):
    # small helper if you want to debug peak amplitudes (not used for std)
    X = np.fft.rfft(x - np.mean(x))
    f = np.fft.rfftfreq(len(x), d=1.0/fs)
    i = np.argmin(np.abs(f - freq))
    return 2.0*np.abs(X[i]) / len(x)

class GearTorqueStdNode(Node):
    def __init__(self):
        super().__init__('gear_torque_std')

        p = self.declare_parameter
        # --- train params (simple 4-gear train with three meshes: 1-2, 2-3, 3-4) ---
        self.z1 = p('z1', 20).get_parameter_value().integer_value
        self.z2 = p('z2', 30).get_parameter_value().integer_value
        self.z3 = p('z3', 40).get_parameter_value().integer_value
        self.z4 = p('z4', 50).get_parameter_value().integer_value
        self.rpm1 = p('rpm1', 600.0).get_parameter_value().double_value
        self.Tin_nom = p('Tin_nom', 1.0).get_parameter_value().double_value

        # efficiencies per mesh
        self.eta12 = p('eta12', 0.97).get_parameter_value().double_value
        self.eta23 = p('eta23', 0.97).get_parameter_value().double_value
        self.eta34 = p('eta34', 0.97).get_parameter_value().double_value

        # misalignments per mesh (deg)
        self.mis12 = p('mis12_deg', 0.0).get_parameter_value().double_value
        self.mis23 = p('mis23_deg', 0.0).get_parameter_value().double_value
        self.mis34 = p('mis34_deg', 0.0).get_parameter_value().double_value

        # ripple baseline & sensitivity
        self.base = p('base_ripple', 0.0015).get_parameter_value().double_value  # set 0.0 for math-perfect
        self.k    = p('k_ripple',    0.15).get_parameter_value().double_value

        # noise/drift toggles
        self.noise_sigma_out = p('noise_sigma_out', 0.0015).get_parameter_value().double_value  # set 0.0 to disable
        self.drift_amp       = p('drift_amp',       0.003).get_parameter_value().double_value    # set 0.0 to disable

        # sim/time
        self.rate_hz = p('rate_hz', 1000.0).get_parameter_value().double_value
        self.dt = 1.0 / self.rate_hz

        # std window & threshold
        self.std_window_sec = p('std_window_sec', 0.2).get_parameter_value().double_value
        self.std_thresh     = p('std_threshold',  0.02).get_parameter_value().double_value  # absolute Nm, or use fraction * Tout_nom

        # publishers
        q = 10
        self.pub_std   = self.create_publisher(Float64, '/gear_torque/std_out', q)
        self.pub_ok    = self.create_publisher(Bool,    '/gear_torque/aligned', q)
        self.pub_thr   = self.create_publisher(Float64, '/gear_torque/threshold', q)

        # rolling buffer for std
        self.buf = deque(maxlen=int(self.std_window_sec * self.rate_hz))

        # precompute RPMs for each gear in a simple train (each on its own shaft):
        # speeds propagate by adjacent pair ratios:
        # rpm2 = rpm1*(z1/z2), rpm3 = rpm2*(z2/z3), rpm4 = rpm3*(z3/z4)
        self.rpm2 = self.rpm1 * (self.z1 / self.z2)
        self.rpm3 = self.rpm2 * (self.z2 / self.z3)
        self.rpm4 = self.rpm3 * (self.z3 / self.z4)

        # overall ratio magnitude and nominal Tout
        ratio = (self.z2/self.z1)*(self.z3/self.z2)*(self.z4/self.z3)  # simplifies to z4/z1
        eta_total = self.eta12 * self.eta23 * self.eta34
        self.Tout_nom = self.Tin_nom * ratio * eta_total

        # mesh frequencies (driver = upstream gear each time)
        self.f12 = self.z1 * (self.rpm1/60.0)
        self.f23 = self.z2 * (self.rpm2/60.0)
        self.f34 = self.z3 * (self.rpm3/60.0)

        # phases (deterministic)
        rng = np.random.RandomState(7)
        self.ph12 = rng.rand(6)*2*np.pi
        self.ph23 = rng.rand(6)*2*np.pi
        self.ph34 = rng.rand(6)*2*np.pi

        self.t = 0.0
        self.timer = self.create_timer(self.dt, self._tick)

    def _amps(self, mis_deg):
        theta = math.radians(mis_deg)
        a1 = self.base + self.k*theta/(1+10*theta)
        return a1, 0.35*a1, 0.25*a1

    def _mesh_ripple(self, t, fmesh, fshaft, a1, a2, aside, ph):
        return ( a1*np.sin(2*np.pi*fmesh*t      + ph[0])
               + a2*np.sin(2*np.pi*2*fmesh*t    + ph[1])
               + aside*np.sin(2*np.pi*(fmesh+fshaft)*t + ph[2])
               + 0.8*aside*np.sin(2*np.pi*(fmesh-fshaft)*t + ph[3])
               + 0.5*aside*np.sin(2*np.pi*(fmesh+2*fshaft)*t + ph[4])
               + 0.5*aside*np.sin(2*np.pi*(fmesh-2*fshaft)*t + ph[5]) )

    def _tick(self):
        t = self.t
        # shaft fundamentals
        fsh1 = self.rpm1/60.0
        fsh2 = self.rpm2/60.0
        fsh3 = self.rpm3/60.0

        # amplitudes from misalignment per mesh
        a1_12, a2_12, as_12 = self._amps(self.mis12)
        a1_23, a2_23, as_23 = self._amps(self.mis23)
        a1_34, a2_34, as_34 = self._amps(self.mis34)

        # synthesize ripples for each mesh (weights heuristic: downstream meshes often dominate)
        r12 = self._mesh_ripple(t, self.f12, fsh1, a1_12, a2_12, as_12, self.ph12)
        r23 = self._mesh_ripple(t, self.f23, fsh2, a1_23, a2_23, as_23, self.ph23)
        r34 = self._mesh_ripple(t, self.f34, fsh3, a1_34, a2_34, as_34, self.ph34)

        w12, w23, w34 = 0.5, 0.7, 1.0

        drift = self.drift_amp * math.sin(2*math.pi*0.2*t + 0.7) if self.drift_amp>0 else 0.0
        noise = self.noise_sigma_out * np.random.randn() if self.noise_sigma_out>0 else 0.0

        Tout = self.Tout_nom * (1.0 + w12*r12 + w23*r23 + w34*r34 + drift) + self.Tout_nom * noise

        # update rolling buffer and publish std and status
        self.buf.append(Tout)
        if len(self.buf) >= 2:
            std_val = float(np.std(self.buf))
            ok = std_val < self.std_thresh
            self.pub_std.publish(Float64(data=std_val))
            self.pub_ok.publish(Bool(data=ok))
            self.pub_thr.publish(Float64(data=self.std_thresh))

        self.t += self.dt

def main():
    rclpy.init()
    node = GearTorqueStdNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
