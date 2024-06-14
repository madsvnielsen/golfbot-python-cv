class CVGrid:

    def __init__(self, grid_size, frame_size) -> None:
        self.grid_size = grid_size
        self.frame_size = frame_size
        self.cell_size = (int(frame_size[0]/grid_size[0]), int(frame_size[1]/grid_size[1]))
        self.cells = [[CVCell((y,x),(x*self.cell_size[0], y*self.cell_size[1]), "unblocked") for x in range(grid_size[0])] for y in range(grid_size[1])]

    def block_at_pixel_position(self, position):
        x, y = position
        for column in self.cells:
            for cell in column:
                cell_x, cell_y = cell.pixel_position
                cell_width, cell_height = self.cell_size
                if cell_x <= x < cell_x + cell_width and cell_y <= y < cell_y + cell_height:
                    cell.status = "blocked"
                    return


    def clearBlocks(self):
        for x in self.cells:
            for y in x:
                y.status = "unblocked"





class CVCell:
    def __init__(self, position, pixel_position, status):
        self.position = position
        self.pixel_position = pixel_position
        self.status = status

    def __str__(self) -> str:
        return "[%s, %s]: %s" % (self.pixel_position[0], self.pixel_position[1], self.status)
