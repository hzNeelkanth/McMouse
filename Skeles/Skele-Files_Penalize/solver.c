#include "solver.h"
#include "API.h"
#include <string.h> // for memset

// Maze arrays
bool walls[N][N][4];
int cost[N][N];
int visited[N][N];

// Robot movement deltas
int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

// Robot state (extern from main)
extern int robot_x;
extern int robot_y;
extern int robot_dir;

// Initialize cost map (goal at center)
void init_cost_map(void) {
    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            cost[x][y] = INF;

    int gx = N / 2 - 1;
    int gy = N / 2 - 1;

    cost[gx][gy] = 0;
    cost[gx][gy+1] = 0;
    cost[gx+1][gy] = 0;
    cost[gx+1][gy+1] = 0;
}

// Check if coordinates are inside the maze
bool valid(int x, int y) {
    return (x >= 0 && x < N && y >= 0 && y < N);
}

// Flood-fill algorithm
void flood_fill(void) {
    bool changed = true;
    while (changed) {
        changed = false;
        for (int x = 0; x < N; x++) {
            for (int y = 0; y < N; y++) {
                for (int d = 0; d < 4; d++) {
                    int nx = x + dx[d];
                    int ny = y + dy[d];
                    if (valid(nx, ny) && !walls[x][y][d]) {
                        if (cost[nx][ny] + 1 < cost[x][y]) {
                            cost[x][y] = cost[nx][ny] + 1;
                            changed = true;
                        }
                    }
                }
            }
        }
    }
}

// Move robot toward next cell (improved logic)
void move_to_next_cell(void) {
    int best_dir = -1;
    int best_cost = INF;

    int back_dir = (robot_dir + 2) % 4; // direction opposite to current

    for (int d = 0; d < 4; d++) {
        int nx = robot_x + dx[d];
        int ny = robot_y + dy[d];

        if (!valid(nx, ny) || walls[robot_x][robot_y][d]) continue;

        // Avoid immediate backtracking unless no other option
        if (d == back_dir) continue;

        // Effective cost
        int effective_cost = cost[nx][ny];
        if (!((nx == N/2 || nx == N/2-1) && (ny == N/2 || ny == N/2-1))) {
            effective_cost += visited[nx][ny];
        }

        // Prefer straight if cost is equal or lower
        if (effective_cost < best_cost ||
            (effective_cost == best_cost && d == robot_dir)) {
            best_cost = effective_cost;
            best_dir = d;
        }
    }

    // If no other option except backtracking, allow it
    if (best_dir == -1) best_dir = back_dir;

    // Turn robot toward best_dir
    int turn = (best_dir - robot_dir + 4) % 4;
    if (turn == 1) API_turnRight();
    else if (turn == 2) { API_turnRight(); API_turnRight(); }
    else if (turn == 3) API_turnLeft();

    // Move forward
    API_moveForward();

    // Update robot state
    robot_x += dx[best_dir];
    robot_y += dy[best_dir];
    robot_dir = best_dir;

    // Mark visited
    visited[robot_x][robot_y]++;

    // Optional visualization
    API_setColor(robot_x, robot_y, 'G');
}

// Update walls from API sensors
bool sense_and_update(void) {
    bool changed = false;

    int left_dir = (robot_dir + 3) % 4;
    int right_dir = (robot_dir + 1) % 4;

    if (API_wallLeft() && !walls[robot_x][robot_y][left_dir]) {
        walls[robot_x][robot_y][left_dir] = true;
        changed = true;
    }
    if (API_wallFront() && !walls[robot_x][robot_y][robot_dir]) {
        walls[robot_x][robot_y][robot_dir] = true;
        changed = true;
    }
    if (API_wallRight() && !walls[robot_x][robot_y][right_dir]) {
        walls[robot_x][robot_y][right_dir] = true;
        changed = true;
    }

    return changed;
}