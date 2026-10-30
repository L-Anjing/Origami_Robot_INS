"""Causal switching-anchor translation with continuous handover; NumPy only."""
import numpy as np


class SwitchingAnchorOdometry:
    """No truth is accepted by this API. A drive event only creates candidates.

    Every activation additionally requires causal stationary evidence and an
    INS-speed check. Without anchors this is exactly the geometry-fusion input.
    At a handover the new anchor's world coordinate is inherited from the old
    root estimate plus the new shape offset, never from a new world origin.
    During gaps, geometry-fusion increments continue from the last root state.
    """
    def __init__(self, config):
        self.config = config
        self.previous_controls = None
        self.previous_fallback = None
        self.root = None
        self.has_anchor_history = False
        self.active = -1
        self.anchor_world = None
        self.anchor_raw_at_acquisition = None
        self.deadline = -np.inf
        self.blocked_until = np.zeros(6)
        self.candidates = []

    def update(self, time, controls, relative_imu_w, fallback_root_w,
               imu_position_w, imu_velocity_w, stationary):
        controls = np.asarray(controls)
        falling = np.zeros(3, dtype=bool) if self.previous_controls is None else ((self.previous_controls == 1) & (controls == 0))
        self.previous_controls = controls.copy()
        previous_active = self.active
        rejected = np.zeros(6, dtype=bool)
        if self.root is None or not self.has_anchor_history:
            root = np.array(fallback_root_w, copy=True)
        else:
            root = self.root + fallback_root_w - self.previous_fallback
        self.previous_fallback = np.array(fallback_root_w, copy=True)
        c = self.config
        if c["enabled"]:
            for drive in np.flatnonzero(falling):
                nodes = c["drive_candidates"][f"u{drive}"]
                if nodes:
                    self.candidates.append((time + c["delay_s"], time + c["candidate_duration_s"], list(nodes)))
            self.candidates = [item for item in self.candidates if item[1] >= time]
            candidates = self.candidates + [(w["start_s"], w["end_s"], [w["node"]]) for w in c["manual_windows"]
                                             if w["start_s"] <= time <= w["end_s"]]
            if self.active >= 0:
                node = self.active
                expired = time > self.deadline
                slipped = (not stationary[node] or np.linalg.norm(imu_velocity_w[node]) > c["reject_speed_m_s"] or
                           np.linalg.norm(imu_position_w[node] - self.anchor_raw_at_acquisition) > c["reject_displacement_m"])
                constrained = self.anchor_world - relative_imu_w[node]
                inconsistent = np.linalg.norm(constrained - root) > c["maximum_innovation_m"]
                if expired or slipped or inconsistent:
                    self.active = -1
                    if not expired:
                        rejected[node] = True
                        self.blocked_until[node] = time + c["rejection_cooldown_s"]
                else:
                    root = constrained
            # Config order resolves simultaneous admissible candidates; not a
            # claim that the three physical drives are mutually exclusive.
            acquired = False
            for start, stop, nodes in sorted(candidates, key=lambda item: item[0], reverse=True):
                if not start <= time <= stop:
                    continue
                for node in nodes:
                    if node == self.active:
                        acquired = True
                        break
                    if time < self.blocked_until[node]:
                        continue
                    if stationary[node] and np.linalg.norm(imu_velocity_w[node]) <= c["accept_speed_m_s"]:
                        self.active = node
                        self.has_anchor_history = True
                        self.anchor_world = root + relative_imu_w[node]
                        self.anchor_raw_at_acquisition = imu_position_w[node].copy()
                        self.deadline = min(stop, time + c["maximum_hold_s"])
                        acquired = True
                        break
                if acquired:
                    break
        self.root = root.copy()
        return {"root_position_W": root, "active_node": self.active,
                "handover": self.active >= 0 and previous_active != self.active,
                "rejected_nodes": rejected, "falling_drives": falling}
