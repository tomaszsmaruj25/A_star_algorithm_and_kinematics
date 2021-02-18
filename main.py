import pygame
import random
import heapq as pq
import heapq
import math
import numpy as np

pygame.init()

# Define window dimension
dis_width = 600
dis_height = 400
dis = pygame.display.set_mode((dis_width, dis_height))
pygame.display.set_caption('A* Algorithm + kinematics')

# Define dimensions in the window
block = 10  # dimension of the A* square path
L = 15  # vehicle dimension - between the wheels
d = R = 20  # vehicle dimension - radian of the wheels

game_over = False  # end simulation condition

# Define basic colors
white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
gray = (128, 128, 128)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)


# Initializing map with obstacles
def empty_map(width=60, height=40):
    grid = []
    obst1 = [14, 14, 6]
    obst2 = [20, 30, 5]
    obst3 = [45, 15, 7]

    for r in range(height):
        row = []
        for c in range(width):
            # Frames around the windows as obstacle
            if (c == 0) or (c == width - 1) or (r == 0) or (r == height - 1):  # the first and the last column obstacle
                row.append(100)
                continue
            if (c < int(R / 10) + 1) or (c > width - int(R / 10) - 2) or (r < int(R / 10) + 1) or (
                    r > height - int(R / 10) - 2):
                row.append(50)
                continue
            # Draw first obstacle and frame around
            if obst1[0] - obst1[2] < c < obst1[0] + obst1[2] and obst1[1] - obst1[2] < r < obst1[1] + obst1[2]:
                row.append(100)
                continue
            if obst1[0] - obst1[2] - int(R / 10) < c < obst1[0] + obst1[2] + int(R / 10) and \
                    obst1[1] - obst1[2] - int(R / 10) < r < obst1[1] + obst1[2] + int(R / 10):
                row.append(50)
                continue
            # Draw second obstacle and frame around
            if obst2[0] - obst2[2] < c < obst2[0] + obst2[2] and obst2[1] - obst2[2] < r < obst2[1] + obst2[2]:
                row.append(100)
                continue
            if obst2[0] - obst2[2] - int(R / 10) < c < obst2[0] + obst2[2] + int(R / 10) and \
                    obst2[1] - obst2[2] - int(R / 10) < r < obst2[1] + obst2[2] + int(R / 10):
                row.append(50)
                continue
            # Draw second obstacle and frame around
            if obst3[0] - obst3[2] < c < obst3[0] + obst3[2] and obst3[1] - obst3[2] < r < obst3[1] + obst3[2]:
                row.append(100)
                continue
            if obst3[0] - obst3[2] - int(R / 10) < c < obst3[0] + obst3[2] + int(R / 10) and \
                    obst3[1] - obst3[2] - int(R / 10) < r < obst3[1] + obst3[2] + int(R / 10):
                row.append(50)
                continue
            row.append(0)
        grid.append(row)
    return grid


map = empty_map()


# Function that draws a map
def draw_map(map, block):
    for idxR, row in enumerate(map):
        for idxC, col in enumerate(row):
            if col == 100:  # When there is an obstacle, draw a black rectangle
                pygame.draw.rect(dis, black, [idxC * block, idxR * block, block, block])
            if col == 50:  # When there is a frame, draw a gray rectangle
                pygame.draw.rect(dis, gray, [idxC * block, idxR * block, block, block])


# Generate point randomly on the map
def generate_point(width, height, map, snake_block):
    pointValid = False
    while not pointValid:
        point_x = round(random.randrange(0, width - snake_block) / snake_block) * snake_block
        point_y = round(random.randrange(0, height - snake_block) / snake_block) * snake_block
        if map[int(point_y / snake_block)][int(point_x / snake_block)] == 0:  # Prevent getting point on the obstacle
            return point_x, point_y


# Draw unicycle on the map
def draw_unicycle(x, y, theta):
    # where (x,y) is the point is between the wheels
    theta_rad = math.radians(theta)  # convert theta in degrees to radians
    # find characteristic points
    left_wheel = (int(x - 0.5 * L * math.sin(theta_rad)), int(y - 0.5 * L * math.cos(theta_rad)))
    right_wheel = (int(x + 0.5 * L * math.sin(theta_rad)), int(y + 0.5 * L * math.cos(theta_rad)))
    z_point = (int(x - math.cos(theta_rad) * R), int(y + math.sin(theta_rad) * R))
    pygame.draw.polygon(dis, red, (z_point, left_wheel, right_wheel), width=0)  # draw triangle


# Function for calculate heuristic distance
def heuristics(st, end):
    # distance = abs(st[0] - end[0]) + abs(st[1] - end[1])  # Manhattan
    distance = ((st[0] - end[0]) ** 2 + (st[1] - end[1]) ** 2) ** 0.5  # Euclidean
    return distance


# Main function of A* Algorithm
def find_path_a_star(start, end):
    came_from = {}
    current = []
    gscore = {start: 0}
    fscore = {start: heuristics(start, end)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == end:
            break
        if map[int(current[1] / block)][int(current[0] / block)] < 50:          # If there is no obstacle
            neighbours = []
            for new in [(0, -block), (0, block), (-block, 0), (block, 0),       # Check for 8-neighbours
                        (block, block), (block, -block), (-block, -block), (-block, block)]:
                position = (current[0] + new[0], current[1] + new[1])
                if int(position[1] / block) >= 40 or int(position[0] / block) >= 60:
                    continue
                else:
                    if map[int(position[1] / block)][int(position[0] / block)] < 50:  # If there is no obstacle
                        neighbours.append(position)

            for neigh in neighbours:                # calculate cost function
                gscore[current] = 1.0               # if the movement is horizontal or vertical
                x_1 = neigh[0] - current[0]
                y_1 = neigh[1] - current[1]
                for i in [-block, block]:
                    for j in [-block, block]:
                        if (x_1, y_1) == (i, j):
                            gscore[current] = 1.4   # if the movement is diagonal

                cost = heuristics(current, neigh) + gscore[current]  # cost of the path
                if cost < gscore.get(neigh, 0) or neigh not in gscore:
                    came_from[neigh] = current
                    gscore[neigh] = cost
                    fscore[neigh] = cost + heuristics(neigh, end)   # cost the neighbour with end point heuristics
                    pq.heappush(oheap, (fscore[neigh], neigh))
    path = []   # create list with all path coords
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path = path[::-1]
    return path  # return path list


def gameLoop():
    global game_over # end condition

    # --------------- INITIAL VALUES --------------
    # Loop refresh rate
    Ts = 0.1  # Seconds
    freq = int(1 / Ts)  # Hz

    # Robot configuration variables
    xp = 305
    yp = 205
    theta_deg = 90
    theta = math.radians(theta_deg)
    q = np.array([[xp],
                  [yp],
                  [theta]])

    # Calculate position of the z point
    xz = xp - d * math.cos(theta)
    yz = yp + d * math.sin(theta)

    # Velocity of the robot
    V = 30

    # Position of the first square a* algorithm
    x1 = int(xz - (xz % block))  # round to 10 the result
    y1 = int(yz - (yz % block))
    start = (x1, y1)

    # Position of the end point
    x_end, y_end = generate_point(dis_width, dis_height, map, block)
    end = (x_end, y_end)
    refresh = 0

    # Main simulation loop
    while not game_over:
        path = find_path_a_star(start, end)  # init A* algorithm for the new points
        print(path)
        step = 0
        # dis.fill(blue)  # Refresh the screen

        # Loop for one path
        while step < len(path):
            # change the a* path when vehicle is in the destined square
            if x1 <= xz < (x1 + block) and y1 <= yz < (y1 + block):
                x1_change = path[step][0] - x1
                y1_change = path[step][1] - y1
                x1 += x1_change
                y1 += y1_change
                step = step + 1  # !!! add condition where step = len(path)

            dis.fill(blue)  # Refresh the screen
            pygame.draw.rect(dis, green, [x_end, y_end, block, block])  # Draw the destination/end point
            pygame.draw.rect(dis, yellow, [x1, y1, block, block])  # Drawing the next square path from A* algorithm

            # ---------- Kinematic model -----------
            # Calculate position of the Z-point
            xz = xp - d * math.cos(theta)
            yz = yp + d * math.sin(theta)

            # Planner - calculate theta and velocity of the Z-point
            (x_dest, y_dest) = (x1 + int(block / 2), y1 + int(block / 2))  # Destined next block (x,y) from A* (center)
            (x_diff, y_diff) = (x_dest - xz, y_dest - yz)  # Difference between the points
            alfa = math.atan2(y_diff, -x_diff)  # Calculate theta of the robot, 180 because of mirror view
            z_dot = np.array([[math.cos(alfa)],
                              [math.sin(alfa)]]) * V  # Velocity of the robot z point

            # Linearization method - calculate velocities Vx and omega
            P = np.array([[math.cos(theta), -d * math.sin(theta)],
                          [math.sin(theta), d * math.cos(theta)]])
            inv_P = np.linalg.inv(P)
            (Vx, omega) = np.dot(inv_P, z_dot)

            # Calculate position change - configuration coordinates q
            g1 = np.array([[-math.cos(theta)],
                           [math.sin(theta)],
                           [0]])
            g2 = np.array([[0],
                           [0],
                           [1]])
            q = q + (g1 * Vx + g2 * omega) * Ts  # calculate new robot coords
            (xp, yp, theta) = q  # new coords of the robot
            theta_deg = math.degrees(theta)  # convert theta to display degrees

            # Print variables
            # print('z_point (x,y): ', xz, yz, ' theta:', theta_deg)  # Print unicycle z_point and theta
            # print('(x_diff, y_diff): ', (x_diff, y_diff))

            # Draw obstacles
            draw_map(map, block)

            # Draw unicycle triangle
            draw_unicycle(xp, yp, theta_deg)

            # Set frequency of the simulation loop
            pygame.time.Clock().tick(freq)
            pygame.display.update()

        # When it finds the end of path
        start = end
        if x1 == x_end and y1 == y_end:
            x_end, y_end = generate_point(dis_width, dis_height, map, block)
            end = (x_end, y_end)


gameLoop()

# ----- variables dictionary ------
# xp, yp - characteristics robot point
# xz, yz - top of the robot (z point)
# x1, y1 - coords of next square of the A* algorithm (left top corner)
# x_dest, y_dest - coords of next square of the A* algorithm (middle of the square)
# x_diff, y_diff - difference between middle of the next A* square and actual Z-point position
# x_end, y_end - coords of final destined point
