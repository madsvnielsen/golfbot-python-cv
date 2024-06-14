from collections import deque


def bfs(grid, start):
    rows, cols = len(grid), len(grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, Down, Left, Right
    queue = deque([start])
    visited = set()
    result = []

    while queue:
        row, col = queue.popleft()

        if (row, col) in visited:
            continue
        visited.add((row, col))

        if 2 <= grid[row][col] <= 11:
            result.append((row, col))

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < rows and 0 <= new_col < cols:
                if (new_row, new_col) not in visited and grid[new_row][new_col] != 0:
                    queue.append((new_row, new_col))

    return result
