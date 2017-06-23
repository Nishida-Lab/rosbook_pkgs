#!/usr/bin/env python

import arrayutil_python_base as base


class Sum(base.ArrayUtil):
  def operate(self):
    return sum(self.array_)

class Ave(base.ArrayUtil):
  def operate(self):
    return sum(self.array_) / len(self.array_)

class Max(base.ArrayUtil):
  def operate(self):
    return max(self.array_)

class Min(base.ArrayUtil):
  def operate(self):
    return min(self.array_)
