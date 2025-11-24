#include "solver.h"
#include "API.h"
#include <string.h>
#include <math.h>

// Maze arrays
bool walls[N][N][4];
int cost[N][N];
int visited[N][N];

// Direction deltas: 0=NORTH,1=EAST,2=SOUTH,3=WEST
int dx[4] = {0, 1, 0, -1};
int dy[4] = {1, 0, -1, 0};

// Robot state (defined in main.c)
extern int robot_x;
extern int robot_y;
extern int robot_dir;


// Initialize cost map (goal at center)

void init_cost_map(void) {
    for (int x = 0; x < N; x++) for (int y = 0; y < N; y++)
     cost[x][y] = INF;

      int gx = (N - 1) / 2;
      int gy = (N - 1) / 2;

    cost[gx][gy] = 0;
    if (N % 2 == 0) {
        cost[gx + 1][gy] = 0;
        cost[gx][gy + 1] = 0;
        cost[gx + 1][gy + 1] = 0;
    } }

// ------------------------------------------------------------
// Check bounds
// ------------------------------------------------------------
bool valid(int x, int y) {
    return (x >= 0 && x < N && y >= 0 && y < N);
}

// ------------------------------------------------------------
// Flood-fill (goal-biased, BFS style)
// ------------------------------------------------------------
void flood_fill(void) {
    int qx[N * N], qy[N * N];
    int front = 0, rear = 0;

    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            cost[x][y] = INF;

    int gx = (N - 1) / 2;
    int gy = (N - 1) / 2;
    cost[gx][gy] = 0;
    qx[rear] = gx; qy[rear++] = gy;

    if (N % 2 == 0) {
        cost[gx + 1][gy] = 0; qx[rear] = gx + 1; qy[rear++] = gy;
        cost[gx][gy + 1] = 0; qx[rear] = gx; qy[rear++] = gy + 1;
        cost[gx + 1][gy + 1] = 0; qx[rear] = gx + 1; qy[rear++] = gy + 1;
    }

    while (front < rear) {
        int x = qx[front];
        int y = qy[front++];
        int base = cost[x][y];

        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d];
            int ny = y + dy[d];
            if (!valid(nx, ny) || walls[x][y][d]) continue;
            if (cost[nx][ny] > base + 1) {
                cost[nx][ny] = base + 1;
                qx[rear] = nx; qy[rear++] = ny;
            }
        }
    }
}

// ------------------------------------------------------------
// Move robot toward lowest-cost neighbor (goal-biased)
// ------------------------------------------------------------
void move_to_next_cell(void) {
    int best_dir = -1;
    double best_score = 1e9;
    double alpha = 0.5; // weight for visited penalty
    int back_dir = (robot_dir + 2) % 4;

    for (int d = 0; d < 4; d++) {
        int nx = robot_x + dx[d];
        int ny = robot_y + dy[d];
        if (!valid(nx, ny) || walls[robot_x][robot_y][d]) continue;

        double score = cost[nx][ny] + alpha * visited[nx][ny];

        if (score < best_score ||
            (fabs(score - best_score) < 1e-6 && d == robot_dir)) {
            best_score = score;
            best_dir = d;
        }
    }

    if (best_dir == -1) best_dir = back_dir;

    int turn = (best_dir - robot_dir + 4) % 4;
    switch (turn) {
        case 1: API_turnRight(); break;
        case 2: API_turnRight(); API_turnRight(); break;
        case 3: API_turnLeft(); break;
        default: break;
    }

    API_moveForward();
    robot_x += dx[best_dir];
    robot_y += dy[best_dir];
    robot_dir = best_dir;

    visited[robot_x][robot_y]++;
    API_setColor(robot_x, robot_y, 'G');
}

// ------------------------------------------------------------
// Sense and update walls (with mirroring)
// ------------------------------------------------------------
bool sense_and_update(void) {
    bool changed = false;
    int left_dir  = (robot_dir + 3) % 4;
    int right_dir = (robot_dir + 1) % 4;

    struct {
        bool detected;
        int dir;
    } sensors[3] = {
        {API_wallLeft(), left_dir},
        {API_wallFront(), robot_dir},
        {API_wallRight(), right_dir}
    };

    for (int i = 0; i < 3; i++) {
        if (sensors[i].detected && !walls[robot_x][robot_y][sensors[i].dir]) {
            walls[robot_x][robot_y][sensors[i].dir] = true;
            int nx = robot_x + dx[sensors[i].dir];
            int ny = robot_y + dy[sensors[i].dir];
            if (valid(nx, ny))
                walls[nx][ny][(sensors[i].dir + 2) % 4] = true; // mirror
            changed = true;
        }
    }

    return changed;
}