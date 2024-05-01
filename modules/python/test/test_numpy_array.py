#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP Python bindings test
#
#############################################################################

import visp
from visp.core import ArrayDouble2D, RotationMatrix, Matrix, HomogeneousMatrix, PoseVector

import numpy as np
import pytest

def test_np_array_modifies_vp_array():
  # Test that numpy is a view of array and that writing to numpy array modifies vpArray
  array = ArrayDouble2D(5, 5, 1.0)
  assert array.getRows() == array.getCols() == 5
  array_np = np.array(array, copy=False)
  assert array_np.shape == (5, 5)
  assert np.all(array_np == 1.0)
  array_np[0:2, 0:2] = 2
  assert array.getMinValue() == 1 and array.getMaxValue() == 2

def fn_test_not_writable_2d(R):
  R_np = np.array(R, copy=False)
  with pytest.raises(ValueError):
    R_np[0, 0] = 1
  with pytest.raises(ValueError):
    R.numpy()[:1] = 0
  with pytest.raises(ValueError):
    row = R[0]
    row[0] = 1
  with pytest.raises(ValueError):
    sub = R[:2, :2]
    sub[0, :] = 1

def test_rotation_matrix_not_writable():
  R = RotationMatrix()
  fn_test_not_writable_2d(R)

def test_homogeneous_matrix_not_writable():
  T = HomogeneousMatrix()
  fn_test_not_writable_2d(T)

def test_numpy_constructor():
  n_invalid = np.array([1, 2, 3])
  with pytest.raises(RuntimeError):
    a = ArrayDouble2D(n_invalid)
  n_valid = np.array([[1, 2, 3], [4, 5, 6]])
  a = ArrayDouble2D(n_valid)
  assert np.all(np.equal(a.numpy(), n_valid))

def test_numpy_constructor_interpreted_as_1d_vector():
  n_1d = np.array([1, 2, 3])
  with pytest.raises(RuntimeError):
    a = ArrayDouble2D(n_1d) # R = 0, c = 0
  ar = ArrayDouble2D(n_1d, r=len(n_1d))
  ac = ArrayDouble2D(n_1d, c=len(n_1d))


def test_numpy_conversion_and_back():
  a = ArrayDouble2D(10, 10, 2.0)
  a_np = a.numpy().copy()
  a2 = ArrayDouble2D(a_np)
  mat = Matrix(a_np)

  for i in range(a.getRows()):
    for j in range(a.getCols()):
      assert a[i, j] == a_np[i, j]
      assert a[i, j] == a2[i,  j]
      assert mat[i, j] == a[i, j]

def test_indexing_array2D():
  a_np = np.asarray([[i for _ in range(10)] for i in range(10)])
  a = ArrayDouble2D(a_np)
  col = list(range(10))
  for i in range(a.getRows()):
    assert np.all(a[i] == float(i))
    assert np.all(a[-i - 1] == float(a.getRows() - i - 1))
    assert np.all(a[:, i] == col)
    assert np.all(a[:, -i - 1] == col)

def test_index_row_not_copy():
  a = ArrayDouble2D(5, 5, 1.0)
  first_row_view = a[0]
  first_row_view[0] = 0.0
  assert a[0, 0] == 0.0

def test_index_slice_not_copy():
  a = ArrayDouble2D(5, 5, 1.0)
  sub_matrix = a[1:3]
  sub_matrix[0] = 0.0
  for i in range(a.getCols()):
    assert a[1, i] == 0.0

def test_index_tuple_not_copy():
  a = ArrayDouble2D(5, 5, 1.0)
  col = a[:, -1]
  col[0] = 0.0
  assert a[0, -1] == 0.0
  sub = a[0:2, 0:2]
  sub[:, :] = 0.0
  for i in range(2):
    for j in range(2):
      assert a[i, j] == 0.0
