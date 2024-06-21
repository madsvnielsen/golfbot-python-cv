import unittest
from io import StringIO
from aStarAlgo import is_valid, is_unblocked, is_destination, calculate_h_value, trace_path, Cell, a_star_search, robot_navigation
from collections import deque
from bfsAlgo import bfs, find_nearest_non_zero
import sys


class AstarAlgoTest(unittest.TestCase):

    def setUp(self):
        self.grid = [[1 for _ in range(180)] for _ in range(120)]
        self.grid[10][10] = 0
        self.grid[11][10] = 0
        self.grid[12][10] = 0

    def test_is_valid(self):
        self.assertTrue(is_valid(0, 0))
        self.assertFalse(is_valid(-1, 0))
        self.assertFalse(is_valid(120, 180))
        self.assertFalse(is_valid(0, 180))

    def test_is_unblocked(self):
        self.assertTrue(is_unblocked(self.grid, 0, 0))
        self.assertFalse(is_unblocked(self.grid, 10, 10))

    def test_is_destination(self):
        self.assertTrue(is_destination(5, 5, (5, 5)))
        self.assertFalse(is_destination(4, 5, (5, 5)))

    def test_calculate_h_value(self):
        self.assertEqual(calculate_h_value(0, 0, (3, 4)), 5.0)
        self.assertEqual(calculate_h_value(1, 1, (4, 5)), 5.0)
        self.assertNotEqual(calculate_h_value(1, 1, (3, 4)), 5.0)

    def test_robot_navigation(self):
        blockArr = [(10, 10), (11, 10), (12, 10)]
        destArr = [(1, 1), (5, 5)]
        srcArr = [(0, 0)]
        inROW = 120
        inCOL = 180

        old_stdout = sys.stdout
        sys.stdout = StringIO()

        try:
            robot_navigation(blockArr, destArr, srcArr, inROW, inCOL)
            output = sys.stdout.getvalue()
            self.assertIn("The destination cell is found", output)
        finally:
            sys.stdout = old_stdout


'''
    def test_trace_path(self):
        cell_details = [[Cell() for _ in range(180)] for _ in range(120)]
        cell_details[1][1].parent_i = 0
        cell_details[1][1].parent_j = 0
        cell_details[0][0].parent_i = 0
        cell_details[0][0].parent_j = 0
        path = trace_path(cell_details, (1, 1))
        self.assertEqual(path, [(0, 0), (1, 1)])

    def test_a_star_search(self):
        src = (0, 0)
        dest = (5, 5)
        self.grid[5][5] = 1
        a_star_search(self.grid, src, dest)
        dest = (10, 10)
        with self.assertRaises(ValueError):
            a_star_search(self.grid, src, dest)
'''


class bfsAlgo(unittest.TestCase):

    def test_basic_case(self):
        grid = [
            [1, 1, 1, 1],
            [1, 3, 0, 1],
            [1, 0, 4, 1],
            [1, 1, 1, 1]
        ]
        start = (1, 1)
        expected = [(1, 1), (2, 2)]
        self.assertEqual(bfs(grid, start), expected)

    def test_no_valid_cells(self):
        grid = [
            [1, 1, 1],
            [1, 0, 1],
            [1, 1, 1]
        ]
        start = (0, 0)
        expected = []
        self.assertEqual(bfs(grid, start), expected)

    def test_all_valid_cells(self):
        grid = [
            [2, 2, 2],
            [2, 2, 2],
            [2, 2, 2]
        ]
        start = (0, 0)
        expected = [(0, 0), (1, 0), (0, 1), (2, 0), (1, 1),
                    (0, 2), (2, 1), (1, 2), (2, 2)]
        self.assertEqual(bfs(grid, start), expected)

    def test_complex_grid(self):
        grid = [
            [0, 12, 3],
            [4, 5, 6],
            [7, 8, 0]
        ]
        start = (0, 2)
        expected = [(0, 2), (1, 2), (1, 1), (2, 1), (1, 0), (2, 0)]
        self.assertEqual(bfs(grid, start), expected)

    def test_single_cell(self):
        grid = [
            [5]
        ]
        start = (0, 0)
        expected = [(0, 0)]
        self.assertEqual(bfs(grid, start), expected)

    def test_large_grid(self):
        grid = [
            [1, 0, 3, 4, 5],
            [6, 0, 0, 7, 8],
            [9, 10, 11, 0, 2],
            [1, 1, 1, 1, 1]
        ]
        start = (0, 2)
        expected = [(0, 2), (0, 3), (1, 3), (0, 4), (1, 4),
                    (2, 4), (2, 2), (2, 1), (2, 0), (1, 0)]
        self.assertEqual(bfs(grid, start), expected)

    def test_start_on_obstacle(self):
        grid = [
            [1, 1, 1],
            [1, 0, 1],
            [1, 1, 1]
        ]
        start = (1, 1)
        expected = []
        self.assertEqual(bfs(grid, start), expected)


class find_non_zero(unittest.TestCase):

    def test_start_non_zero(self):
        grid = [
            [1, 1, 1, 1],
            [1, 3, 0, 1],
            [1, 0, 4, 1],
            [1, 1, 1, 1]
        ]
        start = (1, 1)
        expected = (1, 1)
        self.assertEqual(find_nearest_non_zero(grid, start), expected)

    def test_start_zero(self):
        grid = [
            [1, 1, 1, 1],
            [1, 3, 0, 1],
            [1, 0, 4, 1],
            [1, 1, 1, 1]
        ]
        start = (2, 1)
        expected = (1, 1)
        self.assertEqual(find_nearest_non_zero(grid, start), expected)


if __name__ == '__main__':
    unittest.main()
