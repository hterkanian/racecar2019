#!/usr/bin/env python

"""
Author: Harry Sarkis Terkanian
Date: November 20, 2019

Example altering a mutable global variable inside a function block.
'line_fit' function fits a straight line to four (error, time_stamp) pairs.
"""

import numpy as np

def line_fit(list_in, error, time_stamp):
	"""update list and calculate slope and intercept of a line from list of (y, x) pairs"""
	list_in.pop(0)				#remove the oldest entry
	list_in.append((error, time_stamp))	#add new entry
	y, x = zip(*list_in)			#unzip into two lists (x, y)
	return np.polyfit(x, y, 1)		# fit a straignt line to pairs
						# return is (slope, intercept) tuple

if __name__ == '__main__':
	# history is a mutable list of tuples representing (error, time) pairs
	history = [(1, 0.10), (1.1, 0.12), (1.2, 0.14), (1.3, 0.16)]
	error = 1.4
	error_time = 0.18

	print('history list before function call:')
	print(history)
	print('error: %.2f, error_time: %.2f\n' % (error, error_time))

	line = line_fit(history, error, error_time)
	print('Function call result: line slope: %.2f, intercept: %.2f\n' % (line[0], line[1]))
	print('history list after function call:')
	print(history)
