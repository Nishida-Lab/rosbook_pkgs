#!/usr/bin/env python
from abc import abstractmethod


class ArrayUtil:
  def __init__(self):
    self.array_ = None

  def setArray(self, array):
    self.array_ = array

  @abstractmethod
  def operate(self):
    pass
