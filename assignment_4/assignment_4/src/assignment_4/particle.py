from assignment_3.geometry import *
from math import pi
import random
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

global global_the_map

#-------------------------------------------------------------------------------
# Generates a random pose in the map (in real world coordinates)
def random_particle(the_map):

  global global_the_map
  global_the_map = the_map

  stopWhile = 1

  while stopWhile == 1:

    #generate a random point
    x = random.randrange(0,the_map.info.width)
    y = random.randrange(0,the_map.info.height)
    theta_id = random.randrange(0,1)
    if theta_id == 0:
      theta = random.random()/3.14
    else:
      theta = -(random.random()/3.14)

    #check if point is in free space
    index_value = to_index(x,y,the_map.info.width)
    if the_map.data[index_value] == 0:
      stopWhile = 0
      #convert to world coordinates
      (x,y) = to_world(x, y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
      return (x, y, theta)

#-------------------------------------------------------------------------------
# Generates a new particle from an old one by adding noise to it
def new_particle(particle, spatial_var, angle_var):

  the_map = global_the_map

  stopWhile = 1

  while stopWhile == 1:

    #generate a random point
    x = float(np.random.normal(particle[0],spatial_var,1))
    y = float(np.random.normal(particle[1],spatial_var,1))
    theta = float(np.random.normal(particle[2],angle_var,1))
    theta = theta % pi - pi

    point_grid = to_grid(x,y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
    if point_grid != None:
      (x1,y1) = point_grid
      index_value = int(to_index(x1,y1,the_map.info.width))
      if the_map.data[index_value] == 0:
        stopWhile = 0
        return (x, y, theta)
    
#-------------------------------------------------------------------------------
# Resamples the particles.
# NOTE: particle weights are not normalized i.e. it is not guaranteed that the 
# sum of all particle weights is 1.
def resample(particles_weighted, n_particles):

  the_map = global_the_map

  particles = []
  pdf = []
  pdf_sum = 0
  cdf = []

  for i in range(len(particles_weighted)):
    pdf_sum = pdf_sum + particles_weighted[i][0]

  for j in range(len(particles_weighted)):
    pdf.append(particles_weighted[j][0]/pdf_sum)

  cdf.append(particles_weighted[0][0]/pdf_sum)
  for k in range(1,len(particles_weighted)):
    cdf.append(cdf[k-1] + (particles_weighted[k][0]/pdf_sum) )

  s_variable = 2.0 # percent
  a_variable = 1.0 # fraction of pi
  spatial_var = math.sqrt(the_map.info.width*the_map.info.height*s_variable/100*s_variable/100)
  angle_var = math.pi/a_variable

  #point with highest probability
  max_pdf_index = sorted(range(len(pdf)), key=lambda k: pdf[k])[-1]
  max_pdf_value = pdf[max_pdf_index]
  particles.append(particles_weighted[max_pdf_index][1])
  particle_new = new_particle(particles_weighted[max_pdf_index][1], spatial_var, angle_var)
  particles.append(particle_new)

  #Method 1
  # for u in range(2,len(particles_weighted)):
  #   rand_pnt = random.randrange(0,1)
  #   stopWhile_cdf = 1
  #   v = 0
  #   while stopWhile_cdf == 1:
  #     if v == len(cdf):
  #       cdf_index = v-1
  #       stopWhile_cdf = 0
  #     elif rand_pnt >= cdf[v]:
  #       if v == 0:
  #         cdf_index = v
  #       else:
  #         cdf_index = v-1
  #       stopWhile_cdf = 0
  #     else:
  #       v = v + 1
  #   particle_new = new_particle(particles_weighted[cdf_index][1], spatial_var, angle_var)
  #   particles.append(particle_new)

  #Method 2
  # for i in range(len(pdf)):
  #   pdf_curr_value = pdf[i]
  #   j = int(pdf_curr_value * n_particles * 5)
  #   print 'i, j' ,i ,j
  #   if j > 4:
  #     for k in range(2):
  #       particle_new = new_particle(particles_weighted[i][1], spatial_var, angle_var)
  #       particles.append(particle_new)

  #Method 3
  lower_pdf_value = max_pdf_value - (0.05 * max_pdf_value)
  count_pdf_value = 0
  for i in range(len(pdf)):
    if pdf[i] > lower_pdf_value:
      count_pdf_value = count_pdf_value + 1
      
  num_iterations = int((n_particles / count_pdf_value) * 0.8)

  for i in range(len(pdf)):
    if pdf[i] > lower_pdf_value:
      for j in range(num_iterations):
        particle_new = new_particle(particles_weighted[i][1], spatial_var, angle_var)
        particles.append(particle_new)

  num_new_rand_pnts = abs(n_particles - len(particles))

  for i in range(num_new_rand_pnts):
    particle_new = random_particle(the_map)
    particles.append(particle_new)

  # print particles

  return particles      

# ----------------------------------------------------------------------------
# Draw an occupancy grid
def draw_occupancy_grid(the_map, ax):

  for cellId in range(len(the_map.data)):

    # Get cell grid coordinates
    x = cellId // the_map.info.width
    y = cellId %  the_map.info.width

    # Get cell world coordinates
    (x, y) = to_world ( x, y,
                    the_map.info.origin.position.x,
                    the_map.info.origin.position.y,
                    the_map.info.width, the_map.info.height,
                    the_map.info.resolution)


    # Add patch
    res = the_map.info.resolution
    if the_map.data[cellId] == 100:
      patch = patches.Rectangle ( (x-res/2, y-res/2), res, res, color='k', alpha=0.5)
      ax.add_patch(patch)
    elif the_map.data[cellId] == 0:
      patch = patches.Rectangle ( (x-res/2, y-res/2), res, res, color='b', alpha=0.5)
      ax.add_patch(patch)

  None

# ----------------------------------------------------------------------------
# Draw scored particles. If no scores were assigned to the particles (i.e. all
# particles have score 0) then particles are drawn with score of 0.1
def draw_particles_scored(particles_weighted):

  # Check if scores are unassigned
  scoresAssigned = False
  for particle in particles_weighted:
    if particle[0] != 0.0:
      scoresAssigned  = True

  # Draw particles
  for ptclId in range(len(particles_weighted)):
    (x, y, theta) = particles_weighted[ptclId][1]

    mSize = 0.1
    if scoresAssigned:
      mSize = particles_weighted[ptclId][0] * 20

    plt.plot( [x,  x - math.sin(theta)*0.5],
          [y,  y + math.cos(theta)*0.5],
          'g', linewidth=mSize / 5)
    plt.plot(x, y, 'ro', markersize=mSize, markeredgecolor='r')
  None

#-------------------------------------------------------------------------------
# This function is called each interation after calculating the scores of the
# particles. Use it for debugging
def debug_call(particles_weighted, the_map):

  debug = False

  if not debug:
    return 

  # Initialize figure
  my_dpi = 96
  plt.figure(1, figsize=(800/my_dpi, 800/my_dpi), dpi=my_dpi)
  plt.cla()
  plt.xlim ( the_map.info.origin.position.x, the_map.info.origin.position.x + the_map.info.width )
  plt.ylim ( the_map.info.origin.position.y, the_map.info.origin.position.y + the_map.info.height )
  plt.gca().set_aspect('equal', adjustable='box')
  plt.xlabel('X world')
  plt.xlabel('Y world')
  ax = plt.axes()

  # Draw map
  draw_occupancy_grid(the_map, ax)

  # Draw particles
  draw_particles_scored(particles_weighted)

  # Show plot
  plt.draw()

  pause = True
  if pause:
    k = plt.waitforbuttonpress(1)
    while not k:
      k = plt.waitforbuttonpress(1)
  else:
    plt.waitforbuttonpress(1e-6)

None  