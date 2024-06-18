from bfsAlgo import bfs
from aStarAlgo import a_star_search
from collections import deque

class CVGrid:

    def __init__(self, grid_size, frame_size) -> None:
        self.grid_size = grid_size
        self.frame_size = frame_size
        self.cell_size = (int(frame_size[0]/grid_size[0]), int(frame_size[1]/grid_size[1]))
        self.cells = [[CVCell((y,x),(x*self.cell_size[0], y*self.cell_size[1]), "unblocked") for x in range(grid_size[0])] for y in range(grid_size[1])]


    def block_out_of_bounds(self, boundary):
        def is_point_in_polygon(x, y, poly):
            """Determine if a point (x, y) is inside a polygon defined by poly."""
            n = len(poly)
            inside = False

            px, py = poly[0]
            for i in range(1, n + 1):
                sx, sy = poly[i % n]
                if y > min(py, sy):
                    if y <= max(py, sy):
                        if x <= max(px, sx):
                            if py != sy:
                                xinters = (y - py) * (sx - px) / (sy - py) + px
                            if px == sx or x <= xinters:
                                inside = not inside
                px, py = sx, sy

            return inside
        
        # Get the polygon in pixel coordinates
        poly = [
            boundary["top_left"],
            boundary["top_right"],
            boundary["bottom_right"],
            boundary["bottom_left"]
        ]

        for row in self.cells:
            for cell in row:
                cell_center = self.get_pixel_from_position(cell.position)
                if not is_point_in_polygon(cell_center[1], cell_center[0], poly):
                    cell.status = "blocked"

    
    def block_at_pixel_position(self, position):
        x, y = position
        for column in self.cells:
            for cell in column:
                cell_x, cell_y = cell.pixel_position
                cell_width, cell_height = self.cell_size
                if cell_x <= x < cell_x + cell_width and cell_y <= y < cell_y + cell_height:
                    cell.status = "blocked"
                    return
                
    def get_position_from_pixel(self, position):
        x, y = position
        for column in self.cells:
            for cell in column:
                cell_x, cell_y = cell.pixel_position
                cell_width, cell_height = self.cell_size
                if cell_x <= x < cell_x + cell_width and cell_y <= y < cell_y + cell_height:
                    return cell.position
                
    def get_pixel_from_position(self, position):
        cell_width, cell_height = self.cell_size
        return [int(int(cell_width/2) + cell_width * position[0]), int(int(cell_height/2) + cell_height * position[1])]



    def clearBlocks(self):
        for x in self.cells:
            for y in x:
                y.status = "unblocked"

    def expand_block(self, neighbors):
        to_block = set()
        for row in self.cells:
            for cell in row:
                if cell.status == "blocked":
                    for dy in range(-neighbors, neighbors + 1):
                        for dx in range(-neighbors, neighbors + 1):
                            ny, nx = cell.position[0] + dy, cell.position[1] + dx
                            if 0 <= ny < self.grid_size[1] and 0 <= nx < self.grid_size[0]:
                                to_block.add((ny, nx))
        for ny, nx in to_block:
            self.cells[ny][nx].status = "blocked"



    ## Give destination and source arrays in grid coordinates.
    def navigate(self, destination, source):
        
        # Define the grid (1 for unblocked, 0 for blocked)
        grid = [[1 for _ in range(self.grid_size[0])] for _ in range(self.grid_size[1])]
        
        # coordinater for forhindringer bliver sat til 0 i grid
        for y in range(self.grid_size[0]):
            for x in range(self.grid_size[1]):
                if self.cells[x][y].status == "blocked":
                    grid[x][y] = 0

        value = 3
        for n in destination:
            grid[n[0]][n[1]] = value
            value += 1

        start = tuple(source[0])
        result = bfs(grid, start)
        destArr = result
        if len(destArr) < len(source):
            return [] 
        print(result)
        # kopire alt andet end sidste element til srcArr og beholder robot start position
        source.extend(destArr[:-1])
        # destination til maal
        special_dest = [59, 1]

        path = []

        # while loop for at komme til maal efter x bolde og der efter vidre
        i = 0
        while i < len(source):
            src = source[i]
            if i > 0 and (i + 1) % 6 == 0:  # Check for every sixth iteration (1-indexed)
                print(f"Running A* search from {source[i]} to {special_dest}")
                res = a_star_search(grid, source[i], special_dest)
                path.extend()
                if i + 1 < len(destArr):
                    print(f"Running A* search from {special_dest} to {destArr[i]}")
                    res = a_star_search(grid, special_dest, destArr[i])
                i += 1  # Move to the next source and destination
            else:
                dest = destArr[i]
                print(f"Running A* search from {src} to {dest}")
                res = a_star_search(grid, src, dest)
                i += 1 
            if res is not None:
                path.extend(res)
        return path





class CVCell:
    def __init__(self, position, pixel_position, status):
        self.position = position
        self.pixel_position = pixel_position
        self.status = status

    def __str__(self) -> str:
        return "[%s, %s]: %s" % (self.pixel_position[0], self.pixel_position[1], self.status)
