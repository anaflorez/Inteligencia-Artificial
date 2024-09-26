import sys
import heapq  # Para A*

class Node():
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

    def __lt__(self, other):
        return False  # Se define más adelante en A*

class StackFrontier():
    def __init__(self):
        self.frontier = []

    def add(self, node):
        self.frontier.append(node)

    def contains_state(self, state):
        return any(node.state == state for node in self.frontier)

    def empty(self):
        return len(self.frontier) == 0

    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[-1]
            self.frontier = self.frontier[:-1]
            return node

class QueueFrontier(StackFrontier):
    def remove(self):
        if self.empty():
            raise Exception("empty frontier")
        else:
            node = self.frontier[0]
            self.frontier = self.frontier[1:]
            return node

class Maze():
    def __init__(self, filename):
        with open(filename) as f:
            contents = f.read()

        if contents.count("A") != 1:
            raise Exception("maze must have exactly one start point")
        if contents.count("B") != 1:
            raise Exception("maze must have exactly one goal")

        contents = contents.splitlines()
        self.height = len(contents)
        self.width = max(len(line) for line in contents)

        self.walls = []
        for i in range(self.height):
            row = []
            for j in range(self.width):
                try:
                    if contents[i][j] == "A":
                        self.start = (i, j)
                        row.append(False)
                    elif contents[i][j] == "B":
                        self.goal = (i, j)
                        row.append(False)
                    elif contents[i][j] == " ":
                        row.append(False)
                    else:
                        row.append(True)
                except IndexError:
                    row.append(False)
            self.walls.append(row)

        self.solution = None
        self.num_explored = 0  # Inicializar num_explored aquí

    def print(self):
        solution = self.solution[1] if self.solution is not None else None
        print()
        for i, row in enumerate(self.walls):
            for j, col in enumerate(row):
                if col:
                    print("█", end="")
                elif (i, j) == self.start:
                    print("A", end="")
                elif (i, j) == self.goal:
                    print("B", end="")
                elif solution is not None and (i, j) in solution:
                    print("*", end="")
                else:
                    print(" ", end="")
            print()
        print()

    def neighbors(self, state):
        row, col = state
        candidates = [
            ("up", (row - 1, col)),
            ("down", (row + 1, col)),
            ("left", (row, col - 1)),
            ("right", (row, col + 1))
        ]

        result = []
        for action, (r, c) in candidates:
            if 0 <= r < self.height and 0 <= c < self.width and not self.walls[r][c]:
                result.append((action, (r, c)))
        return result

    def solve_dfs(self):
        return self.solve(StackFrontier())

    def solve_bfs(self):
        return self.solve(QueueFrontier())

    def solve_a_star(self):
        start = Node(state=self.start, parent=None, action=None)
        frontier = []
        heapq.heappush(frontier, (0, start))
        explored = set()
        self.num_explored = 0  # Reiniciar num_explored para A*

        while frontier:
            self.num_explored += 1
            current_cost, node = heapq.heappop(frontier)

            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            explored.add(node.state)

            for action, state in self.neighbors(node.state):
                if state not in explored and not any(child.state == state for _, child in frontier):
                    child = Node(state=state, parent=node, action=action)
                    priority = self.heuristic(state)
                    heapq.heappush(frontier, (current_cost + 1 + priority, child))

        print("No hay solución disponible para el laberinto.")
        return  # Finaliza la función sin lanzar una excepción

    def heuristic(self, state):
        goal_row, goal_col = self.goal
        row, col = state
        return abs(goal_row - row) + abs(goal_col - col)

    def solve(self, frontier):
        start = Node(state=self.start, parent=None, action=None)
        frontier.add(start)
        explored = set()

        while True:
            if frontier.empty():
                raise Exception("no solution")

            node = frontier.remove()
            self.num_explored += 1

            if node.state == self.goal:
                actions = []
                cells = []
                while node.parent is not None:
                    actions.append(node.action)
                    cells.append(node.state)
                    node = node.parent
                actions.reverse()
                cells.reverse()
                self.solution = (actions, cells)
                return

            explored.add(node.state)

            for action, state in self.neighbors(node.state):
                if not frontier.contains_state(state) and state not in explored:
                    child = Node(state=state, parent=node, action=action)
                    frontier.add(child)

def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python maze.py maze.txt")

    m = Maze(sys.argv[1])

    while True:
        print("Seleccione un algoritmo:")
        print("1. DFS")
        print("2. BFS")
        print("3. A*")
        print("4. Salir")

        choice = input("Ingrese su opción (1-4): ")

        if choice == "1":
            print("Resolviendo con DFS...")
            m.solve_dfs()
            break
        elif choice == "2":
            print("Resolviendo con BFS...")
            m.solve_bfs()
            break
        elif choice == "3":
            print("Resolviendo con A*...")
            m.solve_a_star()
            break
        elif choice == "4":
            print("Saliendo...")
            return
        else:
            print("Opción inválida. Intente de nuevo.")

    print("States Explored:", m.num_explored)
    print("Solution:")
    m.print()
    m.output_image("maze.png", show_explored=True)

if __name__ == "__main__":
    main()
