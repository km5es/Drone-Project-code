#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read SITL data files.

@author: kmakhija
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy

file = scipy.fromfile(open("radio1_cal_sequence" +run + ".dat"), dtype=scipy.complex64)

