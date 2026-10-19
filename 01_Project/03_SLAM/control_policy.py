"""Pure velocity selection policy used by the ROS control mux.

Manual input latches control until a new navigation command explicitly resumes it.
An expired command always yields zero; stale Nav2 output never replays.
"""
import math


class ControlPolicy:
    def __init__(self, nav_timeout=0.6, manual_timeout=0.5):
        self.nav_timeout = nav_timeout
        self.manual_timeout = manual_timeout
        self.mode = 'nav'
        self.emergency = False
        self.nav = (0.0, 0.0, float('-inf'))
        self.manual = (0.0, 0.0, float('-inf'))

    @staticmethod
    def _valid(vx, wz):
        return math.isfinite(vx) and math.isfinite(wz)

    def nav_command(self, vx, wz, now):
        if self._valid(vx, wz):
            self.nav = (max(-0.20, min(0.20, vx)),
                        max(-0.80, min(0.80, wz)), now)

    def manual_command(self, vx, wz, now):
        if not self._valid(vx, wz) or self.emergency:
            return
        self.manual = (max(-0.20, min(0.20, vx)),
                       max(-1.0, min(1.0, wz)), now)
        if vx != 0.0 or wz != 0.0:
            self.mode = 'manual'

    def command(self, mode):
        if self.emergency:
            return
        if mode in ('nav', 'stop'):
            self.mode = mode
            if mode == 'nav':
                # Ignore commands Nav2 published before this new task was granted.
                self.nav = (0.0, 0.0, float('-inf'))

    def emergency_stop(self):
        self.emergency = True
        self.mode = 'stop'

    def output(self, now):
        if self.emergency or self.mode == 'stop':
            return 0.0, 0.0
        vx, wz, stamp = self.manual if self.mode == 'manual' else self.nav
        timeout = self.manual_timeout if self.mode == 'manual' else self.nav_timeout
        return (vx, wz) if now - stamp <= timeout else (0.0, 0.0)


def localization_fresh(now_ros, tf_ros, max_age=2.0):
    """TF may be slightly future-dated by slam_toolbox."""
    return tf_ros is not None and -1.0 <= now_ros - tf_ros <= max_age
