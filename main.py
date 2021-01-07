# Ayaz Nakhuda

import pygame as pg
import math
from Queue import PriorityQueue  # note its queue for python3

# from collections import deque


WIDTH = 600
WINDOW = pg.display.set_mode((WIDTH, WIDTH))
pg.display.set_caption("A* Path Finding Algorithm")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


class Node:
    def __init__(self, row, col, width, totalRows):

        self.row = row
        self.col = col
        self.x = row * width  # 900 screen size / 50 squares in total = width, x coordinate
        self.y = col * width  # y coordinate
        self.neighbours = []
        self.width = width
        self.totalRows = totalRows
        self.colour = WHITE

    def getPosition(self):  # This method will return a specific Node's coordinate
        return (self.row, self.col)

    def isLooked(self):
        if self.colour == RED:
            return True

    def isOpen(self):
        if self.colour == GREEN:  # if in the open set
            return True

    def isBarrier(self):  # if barrier
        if self.colour == BLACK:
            return True

    def isStart(self):
        if self.colour == ORANGE:
            return True

    def isEnd(self):
        if self.colour == TURQUOISE:
            return True

    def resetColour(self):
        self.colour = WHITE

    def makeClose(self):
        self.colour = RED

    def makeOpen(self):
        self.colour = GREEN

    def makeBarrier(self):
        self.colour = BLACK

    def makeStart(self):
        self.colour = ORANGE

    def reset(self):
        self.colour = WHITE

    def makeEnd(self):
        self.colour = TURQUOISE

    def makePath(self):
        self.colour = PURPLE

    def draw(self, window):  # Draw node on the grid
        pg.draw.rect(window, self.colour, (self.x, self.y, self.width, self.width))

    def updateNeighbours(self, grid):
        self.neighbours = []

        # down
        if self.row < self.totalRows - 1 and not grid[self.row + 1][
            self.col].isBarrier():  # make sure that the node below is in the grid and is not a barrier
            self.neighbours.append(grid[self.row + 1][self.col])  # add node to the node's list of neighbours.

        # up
        if self.row > 0 and not grid[self.row - 1][
            self.col].isBarrier():  # make sure that the node below is in the grid and is not a barrier
            self.neighbours.append(grid[self.row - 1][self.col])

        # right
        if self.col < self.totalRows - 1 and not grid[self.row][
            self.col + 1].isBarrier():  # make sure that the node below is in the grid and is not a barrier
            self.neighbours.append(grid[self.row][self.col + 1])

        # left
        if self.col > 0 and not grid[self.row][
            self.col - 1].isBarrier():  # make sure that the node below is in the grid and is not a barrier
            self.neighbours.append(grid[self.row][self.col - 1])

    # might not need the below function
    def __lessThan__(self, other):  # compare two nodes
        return False


def makeGrid(rows, width):
    grid = []
    gap = width // rows  # width of the nodes/cubes
    for i in range(rows):
        grid.append([])
        for j in range(rows):  # Send rows again as the grid will be a square
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid


def drawGridLines(WINDOW, rows, width):
    gap = width // rows
    for i in range(rows):
        pg.draw.line(WINDOW, BLACK, (0, i * gap), (width, i * gap))  # use i, this will tell us where to draw the line
        for j in range(rows):
            pg.draw.line(WINDOW, BLACK, (j * gap, 0), (j * gap, width))


def drawEverything(WINDOW, grid, rows, width):
    WINDOW.fill(WHITE)  # fill everything with white for clearing the window

    for row in grid:
        for node in row:
            node.draw(WINDOW)  # calls the draw method in the Node class

    drawGridLines(WINDOW, rows, width)
    pg.display.update()


def getClickedPosition(position, rows, width):
    gap = width // rows
    y, x = position

    row = y // gap  # coordinates of the cube/ Node
    col = x // gap

    return row, col


def heuristic(p1, p2): # This is essentially the distance from the the node to the end node.
    x1, y1 = p1  # coordinates of the start node.
    x2, y2 = p2  # coordinates of the end node.

    return abs(x1 - x2) + abs(y1 - y2)  # Manhattan equation for the distance between two points.


def reconstructPath(cameFrom, current, draw):
    while current in cameFrom:
        current = cameFrom[current]
        current.makePath()
        draw()


def Astar(draw, grid, start, end):
    count = 0
    openSet = PriorityQueue()
    openSet.put((0, count, start))  # weight, the order it was placed and the node itself
    cameFrom = {}
    GScore = {node: float("inf") for row in grid for node in
              row}  # List comprehension used here. Essentially giving all the nodes the initial G score of infinity
    GScore[start] = 0
    FScore = {node: float("inf") for row in grid for node in row}
    FScore[start] = heuristic(start.getPosition(),
                              end.getPosition())  # g score is 0 therefore f score for start node is the heuristic

    openSetHash = {
        start}  # keeps track of all the items in and not in the priority queue.
                # You cannot check if something is in a priority queue but you can with a hash table.

    while not openSet.empty():
        for event in pg.event.get():
            if event.type == pg.QUIT:  # need another way for the user to exit as this is in another while loop.
                pg.quit()

        currentNode = openSet.get()[2]
        openSetHash.remove(
            currentNode)  # synchronize the hash and the queue when removing the current node from the queue.

        if currentNode == end:
            reconstructPath(cameFrom, end, draw)
            end.makeEnd()
            return True

        for neighbour in currentNode.neighbours:
            tempGScore = GScore[currentNode] + 1  # each node has a value of 1. The G score is simply the distance of this node from the start node.

            if tempGScore < GScore[
                neighbour]:  # if the g score of "this" neighbour is less than the g score of the previous neighbour.
                cameFrom[neighbour] = currentNode
                GScore[neighbour] = tempGScore
                FScore[neighbour] = tempGScore + heuristic(neighbour.getPosition(), end.getPosition())
                if neighbour not in openSetHash:
                    count += 1
                    openSet.put((FScore[neighbour], count, neighbour))
                    openSetHash.add(neighbour)
                    neighbour.makeOpen()

        draw()

        if currentNode != start:
            currentNode.makeClose()

    return False  # for when the program has not found a path


def main(window, width):  ### collision checking, start algorithm upon clicking space etc...

    ROWS = 50
    grid = makeGrid(ROWS, width)

    start = None
    end = None

    run = True

    while run:
        drawEverything(window, grid, ROWS, width)
        for event in pg.event.get():  # an event is triggered when the user has pressed a button or clicked their mouse.
            if event.type == pg.QUIT:  # if user presses quit button on window
                run = False

            if pg.mouse.get_pressed()[0]:  # left mouse button
                position = pg.mouse.get_pos()
                row, col = getClickedPosition(position, ROWS, width)
                node = grid[row][col]  # the clicked position becomes a node
                if not start and node != end:
                    start = node
                    start.makeStart()
                elif not end and node != start:
                    end = node
                    end.makeEnd()
                elif node != end and node != start:
                    node.makeBarrier()


            elif pg.mouse.get_pressed()[2]:  # right mouse button

                position = pg.mouse.get_pos()
                row, col = getClickedPosition(position, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pg.KEYDOWN:
                if event.key == pg.K_SPACE and start and end:  # pressing the space key will start the algorithm
                    for row in grid:
                        for node in row:
                            node.updateNeighbours(grid)
                    Astar(lambda: drawEverything(window, grid, ROWS, width), grid, start, end)

                    '''

                    lambda is an anonymous function,
                    ex:
                    x = lambda : print("hello") is equivalent to the function call x() 
                    or ...
                    x = def func():
                       print ("hello")
                       x()
                    benefit of doing this: you can just pass the draw function
                    as a parameter to another function and call it directly 
                    and this is all done on one line.
                    '''

                if event.key == pg.K_c:
                    start = None
                    end = None
                    grid = makeGrid(ROWS, width)

    pg.quit()


main(WINDOW, WIDTH)
