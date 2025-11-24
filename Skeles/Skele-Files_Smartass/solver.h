#ifndef SOLVER_H
#define SOLVER_H
#include <stdbool.h>

#define N 16             // Maze size
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3
#define INF 1000000

// Maze walls: walls[x][y][direction]
extern bool walls[N][N][4];

// Flood-fill cost map
extern int cost[N][N];

// Visited counter for loop avoidance
extern int visited[N][N];

// Movement deltas (NORTH,EAST,SOUTH,WEST)
extern int dx[4];
extern int dy[4];

// Function prototypes
void init_cost_map(void);
void flood_fill(void);
bool valid(int x, int y);
void move_to_next_cell(void);
bool sense_and_update(void);
void flood_fill_from_start(int sx, int sy);
void run_shortest_path_to_start(void);
void run_shortest_path_to_goal(void);

#endif