#include "solver.h"
#include "API.h"
#include <string.h>
#include <math.h>
#include <stdio.h>


bool walls[N][N][4];
int cost[N][N];
int visited[N][N];
int phase = 2;

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
            best_dir = d;   } }

    if (best_dir == -1) best_dir = back_dir;

int turn = (best_dir - robot_dir + 4) % 4;
switch (turn) {
        case 1: API_turnRight(); break;
        case 2: API_turnRight(); API_turnRight(); break;
        case 3: API_turnLeft(); break; }

    if (API_moveForward()) {
        robot_x += dx[best_dir];
        robot_y += dy[best_dir]; } 
        else {
        walls[robot_x][robot_y][best_dir] = true;
        int nx = robot_x + dx[best_dir];
        int ny = robot_y + dy[best_dir];
        if (valid(nx, ny))
            walls[nx][ny][(best_dir + 2) % 4] = true;
        flood_fill();  /* recompute cost map after hitting a wall */
    }

    robot_dir = best_dir;
    visited[robot_x][robot_y]++;
    API_setColor(robot_x, robot_y, 'G');
}


// Sense and update walls (with mirroring)

bool sense_and_update(void) {
    bool changed = false;
    int left_dir  = (robot_dir + 3) % 4;
    int right_dir = (robot_dir + 1) % 4;

  struct {
     bool detected;
    int dir;}sensors[3] = {
      {API_wallLeft(), left_dir},
      {API_wallFront(), robot_dir},
     {API_wallRight(), right_dir}};

 for (int i = 0; i < 3; i++) {
   if (sensors[i].detected && !walls[robot_x][robot_y][sensors[i].dir]) {
      walls[robot_x][robot_y][sensors[i].dir] = true;
     int nx = robot_x + dx[sensors[i].dir];
     int ny = robot_y + dy[sensors[i].dir];
    if (valid(nx, ny))
      walls[nx][ny][(sensors[i].dir + 2) % 4] = true; // mirror
     changed = true;}
}return changed;}

// Reverse flood-fill: compute distance from start cell to all others
// Used to plan the shortest return path

void flood_fill_from_start(int sx, int sy) {
    int qx[N*N], qy[N*N];
    int front = 0, rear = 0;
  for (int x=0; x<N; x++)
  for (int y=0; y<N; y++)
      cost[x][y] = INF;
      cost[sx][sy] = 0;
    qx[rear] = sx; qy[rear++] = sy;

 while (front < rear) {
    int x = qx[front];
    int y = qy[front++];
    int base = cost[x][y];
  for (int d=0; d<4; d++) {
    int nx = x + dx[d];
    int ny = y + dy[d];
   if (!valid(nx, ny) || walls[x][y][d]) 
      continue;
   if (cost[nx][ny] > base + 1) {
      cost[nx][ny] = base + 1;
      qx[rear] = nx; qy[rear++] = ny;}
    }} 
}

// Follow precomputed shortest path to start (Phase 2)     
void run_shortest_path_to_start(void) {
   printf("Starting return-to-start run...\n");

    // Flood-fill using start as goal
   flood_fill_from_start(0, 0);

 while (!(robot_x == 0 && robot_y == 0)) {
    int best_dir = -1;
    int best_cost = INF;

 for (int d = 0; d < 4; d++) {
    int nx = robot_x + dx[d];
    int ny = robot_y + dy[d];
    if (!valid(nx, ny) || walls[robot_x][robot_y][d]) 
      continue;
    if (cost[nx][ny] < best_cost) {
      best_cost = cost[nx][ny];
      best_dir = d;}}

    if (best_dir == -1) {
       printf("No path!\n");
    break;}


    int turn = (best_dir - robot_dir + 4) % 4;
  switch (turn) {
     case 1: API_turnRight(); break;
     case 2: API_turnRight(); API_turnRight(); break;
     case 3: API_turnLeft(); 
    break;}

if (API_moveForward()) {
      robot_x += dx[best_dir];
      robot_y += dy[best_dir]; } 
   else {
      walls[robot_x][robot_y][best_dir] = true;
    int nx = robot_x + dx[best_dir];
    int ny = robot_y + dy[best_dir];
if (valid(nx, ny))
      walls[nx][ny][(best_dir + 2) % 4] = true;
// Recompute flood-fill since the map changed
if(phase == 2)
     flood_fill_from_start(0, 0);
   else
     flood_fill(); } /*Phase 3: from goal*/
      robot_dir = best_dir;
if(phase == 2) API_setColor(robot_x, robot_y, 'Y');
   else API_setColor(robot_x, robot_y, 'C');}
printf("Returned to start!\n");}
     
// Phase 3: Speed run from start to goal using known maze
void run_shortest_path_to_goal(void) 

 {printf("Starting Phase 3: Speed run from start to goal...\n");
// Reset robot to start orientation
      robot_x = 0;robot_y = 0;robot_dir = NORTH;
// Recompute flood-fill costs from goal (since we want to minimize to goal)
init_cost_map(); flood_fill();
    int gx = (N - 1) / 2; int gy = (N - 1) / 2;

bool at_goal = false;
    int steps = 0;

while (!at_goal) {
    int best_dir = -1;
    int best_cost = INF;
  for (int d = 0; d < 4; d++) {
    int nx = robot_x + dx[d];
    int ny = robot_y + dy[d];
if (!valid(nx, ny) || walls[robot_x][robot_y][d])
   continue;
if (cost[nx][ny] < best_cost) {
      best_cost = cost[nx][ny];
      best_dir = d;}}

if (best_dir == -1) {
  printf("No path to goal found!\n");return;}

    int turn = (best_dir - robot_dir + 4) % 4;
  switch (turn) {
    case 1: API_turnRight(); break;
    case 2: API_turnRight(); API_turnRight(); break;
    case 3: API_turnLeft(); break;}
    
if (API_moveForward()) {
      robot_x += dx[best_dir];
      robot_y += dy[best_dir];} 
  else {
      walls[robot_x][robot_y][best_dir] = true;
    int nx = robot_x + dx[best_dir];
    int ny = robot_y + dy[best_dir];
if (valid(nx, ny))
      walls[nx][ny][(best_dir + 2) % 4] = true;
   flood_fill();} robot_dir = best_dir;    /* recompute cost map from goal */


    // Check for goal reach
if (N % 2 == 0)
      at_goal = ((robot_x == gx || robot_x == gx + 1) && (robot_y == gy || robot_y == gy + 1));
  else
      at_goal = (robot_x == gx && robot_y == gy);} printf("Goal reached again in %d steps (speed run)\n", steps); }

