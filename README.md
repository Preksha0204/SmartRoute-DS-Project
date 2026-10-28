# SmartRoute — Intelligent Multi-Criteria City Route Planner

A console-based C mini-project that models a city as a graph and compares routes using common graph traversal and shortest-path algorithms.

## Features

- City-road graph using adjacency lists
- Breadth-First Search (BFS) and Depth-First Search (DFS)
- Dijkstra's shortest-path algorithm with a min-heap
- Route planning strategies for distance, time, cost, and balanced criteria
- Traffic and road-closure simulation
- Route comparison and explanation
- JSON export of the most recent route to `route.json`

## Requirements

- A C compiler such as GCC
- A terminal or command prompt

## Compile

On Linux/macOS or in an environment with GCC:

```bash
gcc smartroute.c -o smartroute -lm
```

## Run

Linux/macOS:

```bash
./smartroute
```

Windows (MinGW GCC):

```bash
gcc smartroute.c -o smartroute.exe -lm
smartroute.exe
```

Follow the menu shown by the program. Use the route-planning option before choosing JSON export, since export saves the latest calculated route.

## Project purpose

This project demonstrates graph representation, traversal, shortest-path search, priority queues, and multi-criteria route selection in C.
