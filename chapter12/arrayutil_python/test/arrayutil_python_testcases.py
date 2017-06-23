#!/usr/bin/env python
import sys
import os
import unittest
import rostest
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../script')
import arrayutil_python_sub as sub

test_array = [1.7, 2.3, 3.2, 4.8]
answer_sum = 12.0
answer_ave = 3.0
answer_min = 1.7
answer_max = 4.8


class ArrayUtilTestSum(unittest.TestCase):
  def runTest(self):
    sum_instance = sub.Sum()
    sum_instance.setArray(test_array)
    result = sum_instance.operate()
    self.assertEquals(result, answer_sum)

class ArrayUtilTestAve(unittest.TestCase):
  def runTest(self):
    ave_instance = sub.Ave()
    ave_instance.setArray(test_array)
    result = ave_instance.operate()
    self.assertEquals(result, answer_ave)

class ArrayUtilTestMin(unittest.TestCase):
  def runTest(self):
    min_instance = sub.Min()
    min_instance.setArray(test_array)
    result = min_instance.operate()
    self.assertEquals(result, answer_min)

class ArrayUtilTestMax(unittest.TestCase):
  def runTest(self):
    max_instance = sub.Max()
    max_instance.setArray(test_array)
    result = max_instance.operate()
    self.assertEquals(result, answer_max)

class ArrayUtilTestSuite(unittest.TestSuite):
  def __init__(self):
    super(ArrayUtilTestSuite, self).__init__()
    self.addTest(ArrayUtilTestSum())
    self.addTest(ArrayUtilTestAve())
    self.addTest(ArrayUtilTestMin())
    self.addTest(ArrayUtilTestMax())

