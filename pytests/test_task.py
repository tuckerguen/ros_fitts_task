import unittest
import numpy as np
import sys
sys.path.append(r"C:\School\thesis\ros_fitts_task\src")
from task import FittsTask


class TestFittsTask(unittest.TestCase):
    def setUp(self):
        self.ws_lims = ((0, 2560), (0, 1440))
        self.target_size_lims = (10, 20)
        self.home_pos = (10, 1440 // 2)
        self.home_size = 10
        self.steps_to_wait = int(1 * 60)
        self.task = FittsTask(workspace_lims=self.ws_lims,
                              target_size_lims=self.target_size_lims,
                              home_pos=self.home_pos,
                              home_size=self.home_size,
                              steps_to_wait=self.steps_to_wait,
                              render=False)

    def test_AllPtsClose_OutlierOutsideWaitRange(self):
        self.task.pointer_pts = [np.array([0, 0])] * (self.steps_to_wait + 10)
        self.task.pointer_pts[:10] = [np.array([1, 1])] * 10
        self.task.stationary_tolerance = 1
        all_pts_close = self.task._all_pts_close()
        self.assertTrue(all_pts_close)

    def test_AllPtsClose_OutlierEqualToTol(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        self.task.pointer_pts[-1] = np.array([1, 1])
        self.task.stationary_tolerance = 1
        all_pts_close = self.task._all_pts_close()
        self.assertTrue(all_pts_close)

    def test_AllPtsClose_OneOutlierLast(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        self.task.pointer_pts[-1] = np.array([1, 1])
        self.task.stationary_tolerance = 0.5
        all_pts_close = self.task._all_pts_close()
        self.assertFalse(all_pts_close)

    def test_AllPtsClose_OneOutlierFirst(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        self.task.pointer_pts[0] = np.array([1, 1])
        self.task.stationary_tolerance = 0.5
        all_pts_close = self.task._all_pts_close()
        self.assertFalse(all_pts_close)

    def test_AllPtsClose_OneOutlierMiddle(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        self.task.pointer_pts[self.steps_to_wait//2] = np.array([1, 1])
        self.task.stationary_tolerance = 0.5
        all_pts_close = self.task._all_pts_close()
        self.assertFalse(all_pts_close)

    def test_AllPtsClose_AllPtsEqual(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        self.task.stationary_tolerance = 1
        all_pts_close = self.task._all_pts_close()
        self.assertTrue(all_pts_close)

    def test_AllPtsClose_NoPointerPts(self):
        self.task.pointer_pts = []
        all_pts_close = self.task._all_pts_close()
        self.assertFalse(all_pts_close)

    def test_AllPtsClose_ExactRightNumberPoints(self):
        self.task.pointer_pts = [np.array([0, 0])] * self.steps_to_wait
        all_pts_close = self.task._all_pts_close()
        self.assertIsNotNone(all_pts_close)

    def test_AllPtsClose_OneTooFewPoints(self):
        self.task.pointer_pts = [0] * (self.steps_to_wait - 1)
        all_pts_close = self.task._all_pts_close()
        self.assertFalse(all_pts_close)

    def test_StepsToWait_WhilePointerInCircle(self):
        for i in range(self.steps_to_wait):
            self.assertTrue(self.task.is_home_state, f"Task registered success {self.steps_to_wait - i} steps early")
            self.task.step(np.array(self.home_pos))
        self.assertFalse(self.task.is_home_state, f"Task did not register success after {self.steps_to_wait} iterations")


    def test_pointer_in_circle(self):
        pass


if __name__ == "__main__":
    unittest.main()
