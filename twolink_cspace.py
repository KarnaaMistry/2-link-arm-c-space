# Karnaa Mistry - CS560 - 2-link arm C-space

import matplotlib . pyplot as plt
import numpy as np
from intersect import Point, intersect

class Arm:
  def __init__(self, x, y, l1, l2):
    self.x = x
    self.y = y
    self.l1 = l1
    self.l2 = l2
    
class Obstacle:
  def __init__(self, points, color):
    self.points = points
    self.color = color

#Draws the initial workspace
def drawEnv(robot, obstacles):
  plt.rcParams["figure.figsize"] = [10, 10]
  plt.grid(False)
  plt.xlim(0,10)
  plt.ylim(0,10)
  plt.xticks(np.arange(0,11))
  plt.yticks(np.arange(0,11))
  colors = ["violet", "dodgerblue", "crimson", "darkgreen", "gold"]

  for o in obstacles:
    for p in range(o.points.size-1):
      plt.plot([o.points[p].x, o.points[p+1].x], [o.points[p].y, o.points[p+1].y], color=colors[o.color])
    plt.plot([o.points[o.points.size-1].x, o.points[0].x], [o.points[o.points.size-1].y, o.points[0].y], color=colors[o.color])

  #plot start and goal arm orientations
  plt.plot([robot.x,robot.x+robot.l1,robot.x+robot.l1], [robot.y,robot.y,robot.y+robot.l2], linewidth = 5, color = "gray")
  plt.plot([goal.x,goal.x,goal.x], [goal.y,goal.y-goal.l1, goal.y-goal.l1-goal.l2], linewidth = 5, color = "cyan")
  plt.plot(robot.x,robot.y, color = "darkcyan", marker=".", markersize = 15)

  plt.show()

#Takes line segment st and determines if it intersects with an obstacle
def intersectObs(s,t,obstacles):
  for o in obstacles:
    for p in range(o.points.size-1):
      if intersect(s, t, o.points[p], o.points[p+1]):
        return o.color
      if intersect(s, t, o.points[o.points.size-1], o.points[0]):
        return o.color
  return -1

#Computes the C-space visualization for the robot, given the obstacle environment
def drawCfree(robot, obstacles):
  pi = np.pi
  plt.rcParams["figure.figsize"] = [10, 10]
  plt.grid(False)
  plt.xlim(0,360)
  plt.ylim(0,360)
  plt.xticks([0,45,90,135,180,225,270,315,360])
  plt.yticks([0,45,90,135,180,225,270,315,360])
  colors = ["violet", "dodgerblue", "crimson", "darkgreen", "gold"]
  l1 = robot.l1
  l2 = robot.l2
  rx = robot.x
  ry = robot.y
 
  D = 1 #discretization factor of Cfree
  for i in range(0,360,D):
    e = Point(rx,ry)
    f = Point(rx + l1*np.cos(i/180*pi), ry + l1*np.sin(i/180*pi))

    col = intersectObs(e,f, obstacles)

    if col != -1: #collision of first arm, a vertical line in Cfree
      plt.vlines(x=i, ymin=0,ymax=359, linewidth = 2, color = colors[col])
    else:
      for j in range(0,360,D):
        g = Point(f.x + l2*np.cos((i+j)/180*pi),  f.y + l2*np.sin((i+j)/180*pi))
        colg = intersectObs(f,g, obstacles)

        if colg != -1:
          plt.scatter(i,j,s=4,color=colors[colg])


  plt.show()
  
    
robot = Arm(5,5,2,2)
goal = Arm(5,5,2,2)
obstacles = [Obstacle(np.asarray([Point(2,5), Point(1.5,5.5), Point(2.5,7), Point(3.5,5.5), Point(3,5)]), 0),
             Obstacle(np.asarray([Point(7,0.5), Point(6,2.5), Point(7,2.5), Point(9.5,1.5)]), 1),
             Obstacle(np.asarray([Point(6.5,7.5), Point(6.5,9.5), Point(7.5,8.5)]), 2),
             Obstacle(np.asarray([Point(4.5,0.5), Point(2,2), Point(4,2.5), Point(4.5,2)]), 3),
             Obstacle(np.asarray([Point(4,7), Point(1.5,8), Point(3.5,9)]), 4)]
obstacles = np.asarray(obstacles)


# drawEnv(robot, obstacles)
# drawCfree(robot, obstacles)