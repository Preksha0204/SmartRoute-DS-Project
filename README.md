# SmartRoute — Intelligent Multi-Criteria City Route Planner

A console-based C data-structures mini-project that models a city road network and helps compare routes using different criteria.

## Features

- Represent locations and roads as a graph using adjacency lists
- Breadth-First Search (BFS) and Depth-First Search (DFS)
- Dijkstra-based shortest-route search using a min-heap
- Route strategies for distance, time, cost, and a balanced score
- Simulate traffic conditions and road closures
- Compare route options and explain the selected route
- Export the latest route to `route.json`

## Requirements

A C compiler such as GCC.

## Compile

```bash
gcc -std=c11 -Wall -Wextra smartroute.c -o smartroute
```

## Run

On Linux/macOS or a terminal with GCC:

```bash
./smartroute
```

On Windows, run the compiled executable:

```text
smartroute.exe
```

Follow the on-screen menu to explore the sample city network and route-planning options. The program can export a JSON file after a route has been calculated.

## Project file

- `smartroute.c` — complete C source code

