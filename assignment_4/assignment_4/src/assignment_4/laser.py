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

  hyp = 1.5 * (sqrt(pow(the_map.info.height,2) + pow(the_map.info.width,2)))
  x1 = x0 + (hyp * cos(angle))
  y1 = y0 + (hyp * sin(angle))
  points = line_seg(x0, y0, x1, y1)

  stopWhile = 1
  k = 1
  pointFound = 0

  while stopWhile == 1:

    index_value = to_index(points[k][0],points[k][1],the_map.info.width)
    
    if the_map.data[index_value] == 100:
      stopWhile = 0
      return points[k]
    elif k == len(points):
      stopWhile = 0
      return None
    else:
      k = k + 1

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

  ranges = []
  start_angle = theta + min_angle

  for i in range(n_readings):

    angleScan = start_angle + (i*increment)
    occupied_point = ray_tracing(x,y,angleScan, the_map)

    if occupied_point != None:
      (x_1, y_1) = to_world(x, y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
      (x_2, y_2) = to_world(occupied_point[0], occupied_point[1], the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
      range_value = sqrt( pow((y_2-y_1),2) + pow((x_2-x_1),2) )

    if range_value > max_range:
      range_value = max_range
      ranges.append(range_value)
    elif range_value != None:
      ranges.append(range_value)
    else:
      ranges.append(max_range)

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

  count_1 = 0.0

  for i in range(len(ranges1)):
  	diff = abs(ranges0[i] - ranges1[i])
  	if diff < 0.5:
  		count_1 = count_1 + 1

  score = (count_1/90)
  return score