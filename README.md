# SmartCity Planner

A Python-based desktop application for smart city planning and management. This tool provides a graphical user interface (GUI) to visualize city nodes, plan routes, schedule tasks using Constraint Satisfaction Problems (CSP), and optimize operations using Simulated Annealing.

## Features

- **Interactive GUI**: Add, remove, and modify city nodes and edges directly on a canvas.
- **Pathfinding**: 
  - **A* Search Algorithm**: Finds the optimal, shortest path between nodes using Euclidean distance as a heuristic.
  - **Greedy Best-First Search**: Quickly finds a path based on heuristic estimates.
- **Task Scheduling (CSP)**: Uses backtracking to resolve Constraint Satisfaction Problems, ensuring tasks don't overlap and follow dependency rules (e.g., Task A must finish before Task B).
- **Schedule Optimization**: Uses Simulated Annealing to optimize task schedules, minimizing constraint penalties and travel times.
- **Undo/Redo Support**: Easily revert or reapply changes made to the city graph.

## Technologies Used

- **Python 3.x**
- **Tkinter**: For the graphical user interface.
- Standard libraries: `math`, `heapq`, `random`, `copy`, `collections`. (No external dependencies required).

## Prerequisites

- Python 3.6 or higher installed on your system.
- Tkinter (usually comes pre-installed with standard Python distributions).

## Installation

1. Clone this repository:
   ```bash
   git clone <your_github_repo_link_here>
   cd <repository_folder>
   ```

2. No extra packages are needed since this project strictly uses Python's standard library.

## Usage

Run the application using Python:

```bash
python smartcity_gui.py
```

### How to Use

1. **Add Node**: Click on the canvas to place a new node or enter coordinates manually.
2. **Add Edge**: Select two nodes to create a connection. The system auto-calculates Euclidean distance.
3. **Set Start/Goal**: Define the starting node and the destination node for pathfinding algorithms.
4. **Run Pathfinding**: Click "Run A*" or "Run Greedy" to visualize the route.
5. **Run CSP / Optimize**: Generate a valid schedule for city tasks or optimize the existing task list using simulated annealing.
6. **Clear / Reset**: Use "Clear All" or load sample data by restarting the application to experiment.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

[MIT](https://choosealicense.com/licenses/mit/)
