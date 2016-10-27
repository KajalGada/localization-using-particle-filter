from math import sin, cos, atan2, hypot, exp, floor, tan, sqrt
from assignment_3.geometry import to_index, to_world, to_grid

# ------------------------------------------------------------------------------
# Given two integer coordinates of two cells return a list of coordinates of
# cells that are traversed by the line segment between the centers of the input
# cells
def line_seg(x0, y0, x1, y1):
  points = []

  # Setup initial conditions
  dx = abs(x1 - x0)
  dy = abs(y1 - y0)
  x = x0
  y = y0
  n = 1 + dx + dy
  x_inc = 1 if x1 > x0 else -1
  y_inc = 1 if  y1 > y0  else -1
  error = dx - dy
  dx *= 2
  dy *= 2

  # Traverse
  while n > 0:
    points.append((x, y))

    if error >= 0:
      x += x_inc
      error -= dy
    else:
      y += y_inc
      error += dx

    n-=1

  # Return
  return points

#-------------------------------------------------------------------------------
# Given a ray find the coordinates of the first occupied cell in a ray
# Parameters:
#   x0, y0    map coordinates of a cell containing ray origin
#   angle     angle of a ray
#   the_map   map
# Return:
#    first occupied cell
def ray_tracing(x0, y0, angle, the_map):

	print 'ray_tracing'

	print 'the_map', the_map

	print 'x0,y0,angle' ,x0,y0,angle
####
	# if angle > 0:

	# 	if angle < 1.57:

	# 		x_ext = the_map.info.width / the_map.info.resolution
	# 		y_ext = the_map.info.height / the_map.info.resolution

	# 	else:

	# 		x_ext = 0
	# 		y_ext = the_map.info.height / the_map.info.resolution

	# else:

	# 	if abs(angle) < 1.57:

	# 		x_ext = the_map.info.width / the_map.info.resolution
	# 		y_ext = 0

	# 	else:

	# 		x_ext = 0
	# 		y_ext = 0

	# #
	# opp_value = abs(x_ext-x0) * tan(angle)
	# adj_value = abs(y_ext-y0) / tan(angle)

	# y_new = y0 + opp_value
	# x_new = x0 + adj_value

	# # print 'x_new,y_new', x_new,y_new

	# if y_new >= 0:

	# 	if y_new <= (the_map.info.height / the_map.info.resolution):

	# 		y1 = y_new
	# 		x1 = x_ext

	# 	else:

	# 		y1 = y_ext
	# 		x1 = x_new

	# else:

	# 	y1 = y_ext
	# 	x1 = x_new

	# # print 'x_ext,y_ext', x_ext,y_ext
	# print 'x1,y1', x1,y1
#####

	hyp = 1.5 * (sqrt(pow(the_map.info.height,2) + pow(the_map.info.width,2)))
  x1 = x0 + (hyp * cos(angle))
  y1 = y0 + (hyp * sin(angle))
  points = line_seg(x0, y0, x1, y1)

	print 'points', points

	# print points[0]
	# print points[0][0]
	# print points[0][1]

	stopWhile = 1
	k = 1

	while stopWhile == 1:

		# if points[k][1] > 1:

		index_value = (points[k][0] - 1) * the_map.info.width / the_map.info.resolution
		index_value = index_value + points[k][1]

		# else:

		# 	index_value = points[k][0]

		index_value = int(index_value)
		# print 'index_value', index_value
		# print 'data point:', the_map.data[index_value]

		if the_map.data[index_value] == 100:

			stopWhile = 0
			pointFound = 1

		elif k == len(points):

			stopWhile = 0

		else:

			k = k + 1

	if pointFound == 1:
		print points[k]
		range_value = sqrt( pow((points[k][1]-y0),2) + pow((points[k][0]-x0),2) )
		print 'range_value',range_value
		return range_value
	else:
		return None

#-------------------------------------------------------------------------------
# Returns a laser scan that the robot would generate from a given pose in a map
# Parameters:
#   x0, y0      robot position in map coordinates
#   theta       robot orientation
#   min_angle   minimum angle of laserscan
#   increment   laserscan increment
#   n_readings  number of readings in a laserscan
#   max_range   maxiimum laser range
#   the_map     map
# Return:
#   array of expected ranges
def expected_scan(x, y, theta, min_angle, increment, n_readings, max_range, the_map):

  print 'expected_scan'
  ranges = []

  print x, y, theta, min_angle, increment, n_readings, max_range

  start_angle = theta + min_angle
  print 'start_angle: ', start_angle

  if start_angle > 3.14:
  	diff_angle = start_angle - 3.14
  	start_angle = -3.14 + diff_angle
  elif start_angle < -3.14:
  	diff_angle = abs(start_angle) - 3.14
  	start_angle = 3.14 - diff_angle

  print 'start_angle: ', start_angle

  for i in range(n_readings):

  	# print 'i', i

  	theta = start_angle + (i*increment)

  	if theta > 3.14:
  		diff_angle = theta - 3.14
  		theta = -3.14 + diff_angle
  	elif theta < -3.14:
  		diff_angle = abs(theta) - 3.14
  		theta = 3.14 - diff_angle

  	range_value = ray_tracing(x,y,theta, the_map)

  	if range_value > max_range:
  		range_value = max_range
  		ranges.append(range_value)
  	elif range_value != None:
  		ranges.append(range_value)

  # print 'the_map is:'
  # print the_map
  # print 'data', len(the_map.data)

  # print 'n_readings', n_readings

  print 'ranges:', ranges

  return ranges

#-------------------------------------------------------------------------------
# Computes the similarity between two laserscan readings.
# Parameters:
#   ranges0     first scan
#   ranges1     second scan
#   max_range   maximum laser range
# Return:
#   similarity score between two scans
def scan_similarity(ranges0, ranges1, max_range):

  print 'scan_similarity'
  # print ranges0
  # print 'length of ranges0', len(ranges0)

  print 'length of ranges0', len(ranges0)
  print 'length of ranges1', len(ranges1)

  print 'ranges0', ranges0 
  print 'ranges1', ranges1

  count_1 = 0.0

  for i in range(len(ranges1)):

  	diff = abs(ranges0[i] - ranges1[i])

  	if diff < 0.5:
  		count_1 = count_1 + 1

  print 'count_1', count_1

  #
  count_2 = 0
  v = len(ranges1) - 1

  for u in range(len(ranges1)):

  	diff = abs(ranges0[u] - ranges1[v])
  	v = v - 1

  	if diff < 0.5:
  		count_2 = count_2 + 1

  print 'count_2', count_2

  score = (count_1/90)

  return score