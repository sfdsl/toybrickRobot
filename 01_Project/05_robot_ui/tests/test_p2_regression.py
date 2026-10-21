import time
import unittest
import os
import tempfile
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid

if Path(__file__).parent.name == 'tests':
    ui = Path(__file__).resolve().parent.parent
    sys.path.insert(0, str(ui))
    sys.path.insert(0, str(ui.parent / '03_SLAM'))

from map_editor import MapEditor
from explore import detect_frontiers, ExploreRunner
from robot_ui.frontier_policy import observation_goals
from state_store import ModeMgr


def message(grid):
    msg = OccupancyGrid()
    msg.info.width, msg.info.height = grid.shape[1], grid.shape[0]
    msg.info.resolution = 0.05
    msg.data = grid.flatten().tolist()
    return msg


class EditorRegression(unittest.TestCase):
    def test_undo_reset_restore_pixels(self):
        grid = np.zeros((30, 30), np.int16)
        grid[10, 10] = 100
        ed = MapEditor()
        ed.load_from_msg(message(grid))
        ed.apply_stroke([(10, 10)], 'erase', 1)
        self.assertEqual(ed.grid[10, 10], 0)
        ed.undo()
        np.testing.assert_array_equal(ed.grid, grid)
        ed.apply_stroke([(20, 20)], 'wall', 2)
        ed.apply_stroke([(10, 10)], 'erase', 1)
        ed.reset()
        np.testing.assert_array_equal(ed.grid, grid)
        self.assertEqual(ed.status()['strokes'], 0)
        self.assertFalse(ed.status()['edited'])


class ExplorationRegression(unittest.TestCase):
    def test_raw_slam_is_separate_from_edited_display(self):
        from map_viewer import MapViewer
        holder = SimpleNamespace(edited_map_msg=None, raw_map_msg=None, map_msg=None)
        raw = message(np.zeros((10, 10), np.int16))
        edited = message(np.full((10, 10), 100, np.int16))
        MapViewer.on_map(holder, raw)
        MapViewer.on_edited_map(holder, edited)
        MapViewer.on_map(holder, raw)
        self.assertIs(holder.raw_map_msg, raw)
        self.assertIs(holder.map_msg, edited)

    def goals(self, grid, robot=(1.25, 2.5), excluded=()):
        return observation_goals(grid, detect_frontiers(grid), 0.05, (0, 0), robot, excluded)

    def test_finished_room_and_wall_crack_do_not_move(self):
        grid = np.full((100, 100), -1, np.int16)
        grid[10:90, 10:90] = 100
        grid[11:89, 11:89] = 0
        grid[30, 30] = -1  # tiny unknown speck
        grid[10, 50] = 0  # one-cell wall crack facing vast unknown
        self.assertEqual(self.goals(grid, (2.5, 2.5)), [])

    def test_open_frontier_gets_reachable_standoff(self):
        grid = np.full((100, 100), -1, np.int16)
        grid[10:90, 10:60] = 0
        grid[10, 10:60] = grid[89, 10:60] = 100
        grid[10:90, 10] = 100
        goals = self.goals(grid, (2.0, 2.5))
        self.assertTrue(goals)
        self.assertTrue(all(g['x'] < 2.85 and g['path'] >= 0.35 for g in goals))
        excluded = [(goals[0]['x'], goals[0]['y'])]
        remaining = self.goals(grid, (2.0, 2.5), excluded)
        self.assertTrue(all(np.hypot(g['x'] - excluded[0][0], g['y'] - excluded[0][1]) >= 0.7
                            for g in remaining))

    def test_disconnected_frontier_is_rejected(self):
        grid = np.full((100, 100), -1, np.int16)
        grid[20:80, 10:40] = 100
        grid[21:79, 11:39] = 0
        grid[20:80, 60:85] = 0
        grid[20, 60:85] = grid[79, 60:85] = 100
        grid[20:80, 60] = 100
        self.assertEqual(self.goals(grid), [])

    def test_localization_mode_is_rejected_before_motion(self):
        node = SimpleNamespace()
        runner = ExploreRunner(node)
        runner._check_mapping_mode = lambda: (False, '当前是定位模式')
        self.assertFalse(runner.go()['ok'])
        self.assertFalse(runner.running)

    def test_explore_blocks_other_motion_until_stop(self):
        modes = ModeMgr()
        self.assertTrue(modes.claim('explore', 'EXPLORE')[0])
        self.assertFalse(modes.claim('web-nav', 'NAV', force=True)[0])
        self.assertFalse(modes.claim('voice', 'TELEOP', force=True)[0])
        sent = []
        runner = ExploreRunner(SimpleNamespace(publish_control=sent.append,
                                               set_cmd=lambda x,y: sent.append((x,y))))
        canceled = []
        runner._goal_handle = SimpleNamespace(cancel_goal_async=lambda: canceled.append(True))
        runner.cancel()
        self.assertTrue(runner.cancel_evt.is_set())
        self.assertEqual(canceled, [True])
        self.assertIn('stop', sent)

    def test_continue_supplies_valid_matching_pose(self):
        from save_map import deserialize_posegraph
        requests = []
        future = SimpleNamespace(done=lambda: True, exception=lambda: None,
                                 result=lambda: SimpleNamespace())
        client = SimpleNamespace(wait_for_service=lambda **kw: True,
                                 call_async=lambda req: (requests.append(req) or future))
        node = SimpleNamespace(create_client=lambda *args: client,
                               robot_pose=lambda **kw: (1.0, 2.0, 0.3))
        with tempfile.TemporaryDirectory() as root:
            prefix = os.path.join(root, 'map')
            open(prefix + '.posegraph', 'w').close()
            self.assertTrue(deserialize_posegraph(node, prefix)[0])
        self.assertEqual(requests[0].match_type, requests[0].START_AT_GIVEN_POSE)
        self.assertEqual(requests[0].initial_pose.x, 1.0)


if __name__ == '__main__':
    unittest.main()
