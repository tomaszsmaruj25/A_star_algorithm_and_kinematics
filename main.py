import pygame
import random
import heapq as pq
import heapq

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
pygame.display.set_caption('A* Algorithm')

block = 10
snake_List = list([])
Length_of_snake = 1
game_over = False


# # Initializing empty map
# def empty_map(width=60, height=40):
#     grid = []
#     for r in range(height):
#         row = []
#         for c in range(width):
#             row.append(0)
#         grid.append(row)
#     return grid


# Initializing empty map
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
        grid[i][12] = 100
        grid[i][22] = 100
        grid[i][32] = 100
        grid[i][42] = 100
        grid[i][52] = 100
    return grid


map = empty_map()


# Function that draws a map
def draw_map(map, block):
    for idxR, row in enumerate(map):
        for idxC, col in enumerate(row):
            if col == 100:  # When there is an obstacle, draw a rectangle
                pygame.draw.rect(dis, black, [idxC * block, idxR * block, block, block])


def generate_food(width, height, map, snake_block, snake_list=[]):
    foodValid = False
    while not foodValid:
        food_x = round(random.randrange(0, width - snake_block) / snake_block) * snake_block
        food_y = round(random.randrange(0, height - snake_block) / snake_block) * snake_block
        if map[int(food_y / snake_block)][int(food_x / snake_block)] == 0:  # Prevent getting food on the obstacle
            if [food_x, food_y] in snake_list:
                foodValid = False  # Prevent getting food on  snake body
                continue

            return food_x, food_y


# Drawing the gray block
def our_block(block, snake_list):
    for x in snake_list:
        pygame.draw.rect(dis, gray, [x[0], x[1], block, block])


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

    x1 = dis_width / 2
    y1 = dis_height / 2

    start = (x1, y1)
    foodx, foody = generate_food(dis_width, dis_height, map, block, snake_List)

    end = (foodx, foody)

    while not game_over:

        path = find_path_a_star(start, end)

        for i in path:
            x1_change = i[0] - x1
            y1_change = i[1] - y1
            x1 += x1_change
            y1 += y1_change

            dis.fill(blue)
            pygame.draw.rect(dis, green, [foodx, foody, block, block])
            snake_Head = []
            snake_Head.append(x1)
            snake_Head.append(y1)
            snake_List.append(snake_Head)
            if len(snake_List) > Length_of_snake:
                del snake_List[0]

            our_block(block, snake_List)
            draw_map(map, block)
            pygame.time.Clock().tick(1)
            pygame.display.update()

        # When it finds the end of path
        start = end
        if x1 == foodx and y1 == foody:
            foodx, foody = generate_food(dis_width, dis_height, map, block, snake_List)
            end = (foodx, foody)
        # game_over=True


gameLoop()
