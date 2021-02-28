# A* is an informed algorithm
# Node refers to a letter, edges connect nodes
# Weighted edge takes a certain amount or length
# Heuristic function is usedto find target and consider best option
# Goal is to find shortest route from A to B
# Start node A
# Openset (queue): Open = {0,A}
# H(n) gives estimate of distance from node n to the end node
# G current shortest distance to get from start rode to a node
# F score is the addition of G score and H score
# F(n) = G(n) + H(n)

# Node F     G   H    Last
#  A   0     0   0
#  B   inf   inf inf
#  C   inf   inf inf
#  D   inf   inf inf

#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
import pygame #importing the pygame library
import math #importing math library
from queue import PriorityQueue #abstract data structure, where each data in queue has a priority

WIDTH = 660 #define width of window
WIN = pygame.display.set_mode((WIDTH, WIDTH)) #set up display
pygame.display.set_caption("A* Path Finding Algorithm") #caption for display
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
#define colors
RED = (255, 0, 0) 
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255) #square not yet looked at
BLACK = (0, 0, 0) #barrier: avoid
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0) 
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
CYAN = (0, 255, 255)
AQUAMARINE = (127, 255, 212)
SPRINGGREEN = (0,255, 127)
MAGENTA = (255, 0, 255)
GOLD = (255, 215, 0)
DODGER = (0, 191, 255)
BLUE2 = (52, 152, 219)
BLUE3 = (0, 213, 241)
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
#define class called spot to build visualizer first
#because algorithm is dependant on visualizer
#hold nodes and values (the cubes) and keep track of neighbors
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
class Spot:
    def __init__(self, row, col, width, total_rows):
        #keep track of coordinate position
        #50 total cubes, screen size 800, 800/50 = 16 width
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE #square not yet looked at
        self.neighbors = [] #empty list
        self.width = width
        self.total_rows = total_rows

    def get_pos(self): #index using row columns
        return self.row, self.col

    def is_closed(self): #have we considered and dumping
        return self.color == BLUE3

    def is_open(self): #are you in the open set
        return self.color == MAGENTA

    def is_barrier(self): #obstacle?
        return self.color == BLACK

    def is_start(self):
        return self.color == GREEN

    def is_end(self):
        return self.color == RED

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = GREEN

    def make_closed(self):
        self.color = BLUE3

    def make_open(self):
        self.color = MAGENTA

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = RED

    def make_path(self):
        self.color = GOLD
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
    def draw(self, win): #draw cubes on screen
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid): #check cube neighbors and see if they're barriers
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): #check if current row is less than total rows - 1. This is going down a row
            self.neighbors.append(grid[self.row + 1][self.col]) #use same column, but add rows if possible and add to neighbor.

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): #check if current row is greater than 0 and self.row -1 to go up and not a barrier. This is going up a row.
            self.neighbors.append(grid[self.row - 1][self.col]) #use same column, but - rows if possible and add to neighbor.

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): #check if current col is less than total rows - 1. This is going right a row.
            self.neighbors.append(grid[self.row][self.col + 1]) #use same row, but add col if possible and add to neighbor.

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): #check if current col is less 0. This is going left a row.
            self.neighbors.append(grid[self.row][self.col - 1]) #use same row, but - col if possible and add to neighbor.

    def __lt__(self, other): #less than, compare two spots
        return False
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
#define heuristic function H()
def h(p1, p2): #using manhattan distance: straight lines
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, current, draw): #end node to start node
    while current in came_from:
        current = came_from[current] #current equal to where we came from
        current.make_path() #make part of path
        draw() #keep drawing
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
def algorithm(draw, grid, start, end):
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start)) #add to priority Queue -> add start node into first node
    came_from = {} #keep track of path, where which nodes came from where
    g_score = {spot: float("inf") for row in grid for spot in row} #start at infinity float. List comprehension, keeps track of current shortest distance from start node to current node
    g_score[start] = 0
    f_score = {spot: float("inf") for row in grid for spot in row} #start at infinity float. List comprehension. Keeps track of predicted distance from current node to end node
    f_score[start] = h(start.get_pos(), end.get_pos()) #heuristic to make an estimate how far start node is from end node

    open_set_hash = {start} #keep track of inside queue

    while not open_set.empty(): #while openset is empty, means every node is considered and path doesnt exist
        for event in pygame.event.get():
            if event.type == pygame.QUIT: #way to let user quit in while loop
                pygame.quit() #quit game

        current = open_set.get()[2] #store f score, count, and node.
        open_set_hash.remove(current) #take node that came out of queue to make sure there are no duplicates

        if current == end: ##at the end, make path
            reconstruct_path(came_from, end, draw)
            end.make_end() #prevent from drawing on end node
            start.make_start() #prevent from drawing on start node
            return True

        for neighbor in current.neighbors: #consider all neighbors of current node
            temp_g_score = g_score[current] + 1 #assume all edges are 1 and calc temp g score
            if temp_g_score < g_score[neighbor]: #if better path
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash: #if neighbor not in openset
                    count += 1 #increment count to add it in
                    open_set.put((f_score[neighbor], count, neighbor)) #put in new neighbor since better path
                    open_set_hash.add(neighbor)
                    neighbor.make_open() #make current neighbor closed since already considered

        draw()

        if current != start: #if current is not the start node, make it closed
            current.make_closed()

    return False
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
#data structure to hold the grid
def make_grid(rows, width):
    grid = []
    gap = width // rows #width is width of entire grid and rows is rows
    #int division gives gaps between each row or width of each cube
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows) #use class to figure out placement
            grid[i].append(spot)

    return grid

#draw gridlines
def draw_grid(win,  rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap)) #draw horizonal lines for each row
        for j in range(rows):
            pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))

#draw everything
def draw(win, grid, rows, width):
    win.fill(WHITE)

    for row in grid:
        for spot in row:
            spot.draw(win)

    draw_grid(win, rows, width)
    pygame.display.update()

def get_clicked_pos(pos, rows, width):
    gap = width // rows #find gaps
    y, x = pos

    row = y // gap #take position and divide by cube width for position
    col = x // gap

    return row,col
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
def main(win, width): #main function to determine all checks
    ROWS = 50 #50 rows
    grid = make_grid(ROWS, width) #generate grid for 2D list

    start =  None #start position
    end = None #end position

    run = True #are we running the main loop
    started = False #algorithm started?
    while run:
        draw(win, grid, ROWS, width) #draw every loop
        for event in pygame.event.get(): #loop through all events and check
            if event.type == pygame.QUIT: #if we quit
                run = False #stop running game

            #if started:  #don't want user to press or change anything but quit button when started
                #continue

            if pygame.mouse.get_pressed()[0]: #if mouse was pressed on left button
                pos = pygame.mouse.get_pos() #gives us position of pygame mouse
                row, col = get_clicked_pos(pos, ROWS, width) #gives us row and column clicked on
                spot = grid[row][col] #index row column in grid
                if not start and spot != end: #make sure start does not equal end
                    start = spot #mark start location if there isnt one
                    start.make_start()

                elif not end and spot != start: #if end doesn't exist + make sure  spot not equal to start
                    end = spot #create end spot
                    end.make_end()

                elif spot != end and spot != start: #if not clicking on start or end
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]: #right button to erase or reset cube
                pos = pygame.mouse.get_pos()  # gives us position of pygame mouse
                row, col = get_clicked_pos(pos, ROWS, width)  # gives us row and column clicked on
                spot = grid[row][col]  # index row column in grid
                spot.reset()
                if spot == start: #reset start node
                    start = None

                elif spot == end: #reset end node
                    end = None

            if event.type == pygame.KEYDOWN: #if down key is pressed
                if event.key == pygame.K_SPACE and start and end: #start running algorithm w/ space key
                    for row in grid:
                        for spot in row:
                            spot.update_neighbors(grid) #update all neighbors

                    algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end) #call algorithm function. Lambda is anonymous function

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit() #exits pygame window
#--------------------------------------------------------------------------------------------------##--------------------------------------------------------------------------------------------------#
main(WIN, WIDTH) #end of program