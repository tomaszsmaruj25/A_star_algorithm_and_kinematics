import pygame
import random
import heapq as pq
import heapq
import math

pygame.init()

white = (255, 255, 255)
yellow = (255, 255, 102)
black = (0, 0, 0)
gray = (128, 128, 128)
red = (213, 50, 80)
green = (0, 255, 0)
blue = (50, 153, 213)

dis_width = 600
dis_height = 400

dis = pygame.display.set_mode((dis_width, dis_height))
pygame.display.set_caption('A* Algorithm + kinematics')

block = 10  # dimension of the A* square path
L = 15  # vehicle dimension - between the wheels
R = 20  # vehicle dimension - radian of the wheels

game_over = False  # end simulation condition


# # Initializing empty map
# def empty_map(width=60, height=40):
#     grid = []
#     for r in range(height):
#         row = []
#         for c in range(width):
#             row.append(0)
#         grid.append(row)
#     return grid


# Initializing map with obstacles
def empty_map(width=60, height=40):
    grid = []
    for r in range(height):
        row = []
        for c in range(width):
            if (c == 0) or (c == width - 1):  # the first and the last column is an obstacle
                row.append(100)
                continue
            if (r == 0) or (r == height - 1):  # the first and the last row is an obstacle
                row.append(100)
                continue
            row.append(0)
        grid.append(row)
    # example of an obstacle
    for i in range(10, 30):
        grid[i][10] = 100
        grid[i][20] = 100
        #grid[i][30] = 100
        grid[i][40] = 100
        grid[i][50] = 100
    return grid


map = empty_map()


# Function that draws a map
def draw_map(map, block):
    for idxR, row in enumerate(map):
        for idxC, col in enumerate(row):
            if col == 100:  # When there is an obstacle, draw a rectangle
                pygame.draw.rect(dis, black, [idxC * block, idxR * block, block, block])


def generate_food(width, height, map, snake_block):
    foodValid = False
    while not foodValid:
        food_x = round(random.randrange(0, width - snake_block) / snake_block) * snake_block
        food_y = round(random.randrange(0, height - snake_block) / snake_block) * snake_block
        if map[int(food_y / snake_block)][int(food_x / snake_block)] == 0:  # Prevent getting food on the obstacle
            return food_x, food_y


def unicycle(x, y, theta):
    # where (x,y) is the point is between the wheels
    theta_rad = math.radians(theta)  # convert theta in degrees to radians
    # find characteristic points
    left_wheel = (int(x - 0.5 * L * math.sin(theta_rad)), int(y - 0.5 * L * math.cos(theta_rad)))
    right_wheel = (int(x + 0.5 * L * math.sin(theta_rad)), int(y + 0.5 * L * math.cos(theta_rad)))
    z_point = (int(x - math.cos(theta_rad) * R), int(y + math.sin(theta_rad) * R))
    # draw the vehicle
    pygame.draw.polygon(dis, red, (z_point, left_wheel, right_wheel), width=0)
    return z_point


def heuristics(st, end):
    # (x0, y0) -> start
    # (x1, y1) -> end
    # ((x1 - x0) ^2 + (y1-y0)^2) ^0.5
    # distance = abs(st[0] - end[0]) + abs(st[1] - end[1])  # Manhattan
    distance = ((st[0] - end[0]) ** 2 + (st[1] - end[1]) ** 2) ** (0.5)  # Euclidean
    return distance


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
        if map[int(current[1] / block)][int(current[0] / block)] != 100:
            neighbours = []

            for new in [(0, -block), (0, block), (-block, 0), (block, 0),
                        (block, block), (block, -block), (-block, -block), (-block, block)]:
                position = (current[0] + new[0], current[1] + new[1])

                if int(position[1] / block) >= 40 or int(position[0] / block) >= 60:
                    continue
                else:
                    if map[int(position[1] / block)][int(position[0] / block)] != 100:
                        neighbours.append(position)

            for neigh in neighbours:
                gscore[current] = 1.0
                x_1 = neigh[0] - current[0]
                y_1 = neigh[1] - current[1]
                for i in [-block, block]:
                    for j in [-block, block]:
                        if (x_1, y_1) == (i, j):
                            gscore[current] = 1.4

                cost = heuristics(current, neigh) + gscore[current]  # cost of the path
                if cost < gscore.get(neigh, 0) or neigh not in gscore:
                    came_from[neigh] = current
                    gscore[neigh] = cost
                    fscore[neigh] = cost + heuristics(neigh, end)
                    pq.heappush(oheap, (fscore[neigh], neigh))

    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path = path[::-1]

    return path


def gameLoop():
    global game_over

    # robot configuration variables
    xp = 305
    yp = 205
    theta = 90

    # calculate position of the z point
    xz = int(xp - math.cos(math.radians(theta)) * R)
    yz = int(yp + math.sin(math.radians(theta)) * R)

    # position of the first square a* algorithm
    x1 = int(xz - (xz % block))  # round to 10 the result
    y1 = int(yz - (yz % block))

    # position of the end point
    start = (x1, y1)
    foodx, foody = generate_food(dis_width, dis_height, map, block)
    end = (foodx, foody)

    while not game_over:

        path = find_path_a_star(start, end)  # init A* algorithm for the new points
        print(path)
        step = 0

        while step < len(path):
            # change the a* path when vehicle is in the destined square
            if x1 <= xz < (x1 + block) and y1 <= yz < (y1 + block):
                x1_change = path[step][0] - x1
                y1_change = path[step][1] - y1
                x1 += x1_change
                y1 += y1_change
                step = step + 1 # !!! add condition where step = len(path)

            dis.fill(blue)  # Refresh the screen
            pygame.draw.rect(dis, green, [foodx, foody, block, block])  # Draw the destination/end point
            pygame.draw.rect(dis, gray, [x1, y1, block, block])  # Drawing the next square path from A* algorithm

            # destined next block (x,y) from A*
            (x_dest, y_dest) = (x1 + int(block/2), y1 + int(block/2))

            # difference between the points
            (x_diff, y_diff) = (x_dest - xp, y_dest - yp)
            # calculate theta of the robot, 180 because of mirror view
            theta = math.degrees(math.atan2(y_diff, -x_diff))

            math.
            # move towards the destined point
            if x_diff > 0:
                xp = xp + 1
            if x_diff < 0:
                xp = xp - 1
            if y_diff > 0:
                yp = yp + 1
            if y_diff < 0:
                yp = yp - 1

            # unicycle(x1, y1, theta)
            xz, yz = unicycle(xp, yp, theta)  # Draw unicycle triangle and return z_point of vehicle
            print('z_point (x,y): ', xz, yz, ' theta:         ', theta)  # Print unicycle z_point and theta
            # print('x_dest, y_dest: ', x_dest, y_dest)
            print('diff: ', x_diff, y_diff)
            draw_map(map, block)

            # set frequency of the simulation loop
            pygame.time.Clock().tick(10)
            pygame.display.update()

        # When it finds the end of path
        start = end
        if x1 == foodx and y1 == foody:
            foodx, foody = generate_food(dis_width, dis_height, map, block)
            end = (foodx, foody)

        # End simulation when ESC pressed
        # pygame.event.get()
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_ESCAPE]:
        #     print("ESC został wcisniety")
        #     game_over = True


gameLoop()

# ----- variables dictionary ------
# xp, yp - characteristics robot point
# xz, yz - top of the robot
# x1, y1 - coords of next square of the A* algorithm (left top corner)
# x_dest, y_dest - coords of next square of the A* algorithm (middle of the square)
