# Python program for A* Search Algorithm
from bfsAlgo import bfs
import math
import heapq

# Define the Cell class


class Cell:
    def __init__(self):
      # Parent cell's row index
        self.parent_i = 0
    # Parent cell's column index
        self.parent_j = 0
 # Total cost of the cell (g + h)
        self.f = float('inf')
    # Cost from start to this cell
        self.g = float('inf')
    # Heuristic cost from this cell to destination
        self.h = 0


# Define the size of the grid
ROW = 180
COL = 120

# Check if a cell is valid (within the grid)


def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

# Check if a cell is unblocked


def is_unblocked(grid, row, col):
    return 1 <= grid[row][col] <= 20

# Check if a cell is the destination


def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# Calculate the heuristic value of a cell (Euclidean distance to destination)


def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# Trace the path from source to destination


def trace_path(cell_details, dest):
    print("The Path is ")
    path = []
    row = dest[0]
    col = dest[1]

    # Trace the path from destination to source using parent cells
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col

    # Add the source cell to the path
    path.append((row, col))
    # Reverse the path to get the path from source to destination
    path.reverse()

    # Print the path
    for i in path:
        print("->", i, end=" ")
    print()

# Implement the A* search algorithm


def a_star_search(grid, src, dest):
    # Check if the source and destination are valid
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        print("Source or destination is invalid")
        return

    # Check if the source and destination are unblocked
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        print("Source or the destination is blocked")
        return

    # Check if we are already at the destination
    if is_destination(src[0], src[1], dest):
        print("We are already at the destination")
        return

    # Initialize the closed list (visited cells)
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    # Initialize the details of each cell
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Initialize the start cell details
    i = src[0]
    j = src[1]
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j

    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j))

    # Initialize the flag for whether destination is found
    found_dest = False

    # Main loop of A* search algorithm
    while len(open_list) > 0:
        # Pop the cell with the smallest f value from the open list
        p = heapq.heappop(open_list)

        # Mark the cell as visited
        i = p[1]
        j = p[2]
        closed_list[i][j] = True

        # For each direction, check the successors
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0),
                      (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dir in directions:
            new_i = i + dir[0]
            new_j = j + dir[1]

            # If the successor is valid, unblocked, and not visited
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                # If the successor is the destination
                if is_destination(new_i, new_j, dest):
                    # Set the parent of the destination cell
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    print("The destination cell is found")
                    # Trace and print the path from source to destination
                    trace_path(cell_details, dest)
                    found_dest = True
                    return
                else:
                    # Calculate the new f, g, and h values
                    g_new = cell_details[i][j].g + 1.0
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new

                    # If the cell is not in the open list or the new f value is smaller
                    if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                        # Add the cell to the open list
                        heapq.heappush(open_list, (f_new, new_i, new_j))
                        # Update the cell details
                        cell_details[new_i][new_j].f = f_new
                        cell_details[new_i][new_j].g = g_new
                        cell_details[new_i][new_j].h = h_new
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j

    # If the destination is not found after visiting all cells
    if not found_dest:
        print("Failed to find the destination cell")


# Driver Code
# srcArr is robot position best guess it should be getRobotPosition(cv, boundrypixel)
# Guess we should get the center of the robot
# blockArr is the coordinates where robot can't drive
# destArr is the coordinates of the balls
def robot_navigation(blockArr, destArr, srcArr):
    # Define the grid (1 for unblocked, 0 for blocked)
    grid = [[1 for _ in range(COL)] for _ in range(ROW)]

    # coordinater for forhindringer bliver sat til 0 i grid
    for i in blockArr:
        grid[i[0]][i[1]] = 0

    value = 3
    for n in destArr:
        grid[n[0]][n[1]] = value
        value += 1

    start = tuple(srcArr[0])
    result = bfs(grid, start)
    destArr = result
    print(result)
    # kopire alt andet end sidste element til srcArr og beholder robot start position
    srcArr.extend(destArr[:-1])
    # destination til maal
    special_dest = [59, 1]

    # while loop for at komme til maal efter x bolde og der efter vidre
    i = 0
    while i < len(srcArr):
        src = srcArr[i]
        if i > 0 and (i + 1) % 6 == 0:  # Check for every sixth iteration (1-indexed)
            print(f"Running A* search from {srcArr[i]} to {special_dest}")
            a_star_search(grid, srcArr[i], special_dest)
            if i + 1 < len(destArr):
                print(f"Running A* search from {special_dest} to {destArr[i]}")
                a_star_search(grid, special_dest, destArr[i])
            i += 1  # Move to the next source and destination
        else:
            dest = destArr[i]
            print(f"Running A* search from {src} to {dest}")
            a_star_search(grid, src, dest)
            i += 1


if __name__ == "__main__":
    blockArr = []
    # coordinater for forhindringer paa bane
    n = int(input("Enter the number of blocked cells: "))
    for i in range(n):
        x = int(input("Enter the x coordinate of the blocked cell: "))
        y = int(input("Enter the y coordinate of the blocked cell: "))
        blockArr.append([x, y])
    destArr = []
    # coordinater for bolde
    n = int(input("Enter the number of balls to collect"))
    for i in range(n):
        x = int(input("Enter the x coordinate of the ball: "))
        y = int(input("Enter the y coordinate of the ball: "))
        destArr.append([x, y])
    srcArr = []
    # Robots start position
    print("Enter the coordinates of the Robot")
    x = int(input("Enter the x coordinate of the robot: "))
    y = int(input("Enter the y coordinate of the robot: "))
    srcArr.append([x, y])
    asd = is_valid(120, 180)
    if asd is False:
        print("Fuck")
    robot_navigation(blockArr, destArr, srcArr)
