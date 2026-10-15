"""控制权状态：ModeMgr 模式互斥（IDLE/TELEOP/NAV/PATROL）"""
import threading
import time


class ModeMgr:
    """控制权互斥：同一时刻只有一种来源能动 /cmd_vel 或发导航目标。

    IDLE   空闲
    TELEOP 网页摇杆（3s 无指令自动过期释放）
    NAV    点选目标 / 多点导航（goal_clear / wp_clear 显式释放）
    PATROL 巡逻执行（patrol/cancel 显式释放；Task 6 接入）

    规则：IDLE 或同模式可认领；跨模式需 force=True，否则拒绝。
    安全动作（/stop、/goal/clear、/wp/clear）不经过本管理器，永不设防。
    """

    # NAV 300s 兜底过期：点选导航完成/放弃后忘记「清空」时自动释放，
    # 防止 PATROL/其它来源被永久拒绝（正常路径仍由 goal_clear/wp_clear/_wp_watch 显式释放）
    TIMEOUT = {'TELEOP': 3.0, 'NAV': 300.0, 'PATROL': None}

    def __init__(self):
        self.lock = threading.Lock()
        self.mode, self.holder, self.t0 = 'IDLE', None, 0.0

    def claim(self, who, mode, force=False, force_unless=None):
        """认领控制权。返回 (ok, msg)；失败时 msg 说明被谁占用

        force_unless：当前模式为该值时**即使 force=True 也拒绝**
        （把"能否抢占"的判断收进锁内，避免快照 TOCTOU）。
        """
        with self.lock:
            self._expire()
            if force_unless is not None and self.mode == force_unless:
                return False, '%s 被 %s 占用（%s）' % (mode, self.mode, self.holder)
            if self.mode not in ('IDLE', mode) and not force:
                return False, '%s 被 %s 占用（%s）' % (mode, self.mode, self.holder)
            self.mode, self.holder, self.t0 = mode, who, time.time()
            return True, 'ok'

    def release(self, who):
        """持有者显式释放；不是持有者则不动"""
        with self.lock:
            if self.holder == who:
                self.mode, self.holder = 'IDLE', None
            return self.mode

    def _expire(self):
        t = self.TIMEOUT.get(self.mode)
        if t and time.time() - self.t0 > t:
            self.mode, self.holder = 'IDLE', None

    def snapshot(self):
        with self.lock:
            self._expire()
            return {'mode': self.mode, 'holder': self.holder}


FEATURES = []          # [{'name','state','detail','href'}]：功能注册表（hub 页渲染）


def register_feature(name, state, detail='', href=''):
    """功能向后端登记：state ∈ 已具备/进行中/待做；已存在同名则更新"""
    for f in FEATURES:
        if f['name'] == name:
            f.update(state=state, detail=detail, href=href)
            return
    FEATURES.append({'name': name, 'state': state, 'detail': detail, 'href': href})
