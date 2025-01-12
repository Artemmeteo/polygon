import numpy as np
import unittest

from increasing_subseq import get_len_longest_subseq


class TestLongestSubseq(unittest.TestCase):

    def test_empty_arr(self):
        self.assertEqual(get_len_longest_subseq([]), 0)

    def test_single_elem(self):
        self.assertEqual(get_len_longest_subseq([5]), 1)

    def test_full_increasing(self):
        self.assertEqual(get_len_longest_subseq([1, 2, 3, 4, 5]), 5)

    def test_full_decreasing(self):
        self.assertEqual(get_len_longest_subseq([5, 4, 3, 2, 1]), 1)

    def test_mixed_sequence(self):
        self.assertEqual(get_len_longest_subseq([1, 3, 2, 4, 5, 3, 4, 6]), 3)

    def test_diff_increasing_subseq(self):
        self.assertEqual(get_len_longest_subseq([1, 2, 1, 2, 3, 1, 2]), 3)

    def test_all_same_elements(self):
        self.assertEqual(get_len_longest_subseq([7, 7, 7, 7, 7]), 1)


if __name__ == "__main__":
    unittest.main()