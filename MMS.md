gcc -o a.out main.c API.c solver.c
./a.out



In solver.c (below existing funcs.):


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
            if (!valid(nx, ny) || walls[x][y][d]) continue;
            if (cost[nx][ny] > base + 1) {
                cost[nx][ny] = base + 1;
                qx[rear] = nx; qy[rear++] = ny;
            }
        }
    }
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
            if (!valid(nx, ny) || walls[robot_x][robot_y][d]) continue;
            if (cost[nx][ny] < best_cost) {
                best_cost = cost[nx][ny];
                best_dir = d;
            }
        }

        if (best_dir == -1) {
            printf("No path back to start!\n");
            return;
        }

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

        API_setColor(robot_x, robot_y, 'Y'); // Mark return path
    }

    printf("Returned to start!\n");
      }



Solver.h (adding two new prototypes):

      void flood_fill_from_start(int sx, int sy);void run_shortest_path_to_start(void);

updated Main.c:


    #include <stdio.h>
    #include <string.h>
    #include "API.h"
    #include "solver.h"

// Robot state
   
    int robot_x = 0;
    int robot_y = 0;
    int robot_dir = NORTH;

   
    int main(void) {
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));

    init_cost_map();
    flood_fill();

    printf("Phase 1: Exploring maze...\n");
    int step = 0;

    // --- Phase 1: Exploration ---
    while (1) {
        sense_and_update();
        flood_fill();
        move_to_next_cell();
        step++;

        int gx = (N - 1) / 2;
        int gy = (N - 1) / 2;
        bool at_goal = false;

        if (N % 2 == 0)
            at_goal = ((robot_x == gx || robot_x == gx + 1) &&
                       (robot_y == gy || robot_y == gy + 1));
        else
            at_goal = (robot_x == gx && robot_y == gy);

        if (at_goal) {
            API_setColor(robot_x, robot_y, 'B');
            printf("Goal reached in %d steps at (%d,%d)\n",
                   step, robot_x, robot_y);
            break;
        }

        if (step > 2000) {
            printf("Exploration exceeded 2000 steps.\n");
            break;
        }
    }

    // --- Phase 2: Return run ---
    run_shortest_path_to_start();

    return 0;
      }


A third phase will handle the final optimal path

Solver.c (below everything else):


// Phase 3: Speed run from start to goal using known maze

     
       void run_shortest_path_to_goal(void) {  
        printf("Starting Phase 3: Speed run from start to goal...\n");

    // Reset robot to start orientation
    robot_x = 0;
    robot_y = 0;
    robot_dir = NORTH;

    // Recompute flood-fill costs from goal (since we want to minimize to goal)
    init_cost_map();
    flood_fill();

    int gx = (N - 1) / 2;
    int gy = (N - 1) / 2;

    bool at_goal = false;
    int steps = 0;

    while (!at_goal) {
        int best_dir = -1;
        int best_cost = INF;

        for (int d = 0; d < 4; d++) {
            int nx = robot_x + dx[d];
            int ny = robot_y + dy[d];
            if (!valid(nx, ny) || walls[robot_x][robot_y][d]) continue;
            if (cost[nx][ny] < best_cost) {
                best_cost = cost[nx][ny];
                best_dir = d;
            }
        }

        if (best_dir == -1) {
            printf("No path to goal found!\n");
            return;
        }

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
        steps++;

        API_setColor(robot_x, robot_y, 'C'); // Mark speed-run path

        // Check if we reached the goal (odd or even maze)
        if (N % 2 == 0)
            at_goal = ((robot_x == gx || robot_x == gx + 1) &&
                       (robot_y == gy || robot_y == gy + 1));
        else
            at_goal = (robot_x == gx && robot_y == gy);
    }

    printf("Goal reached again in %d steps (speed run)\n", steps);
     }


solver.h (added prototype):
 
     void run_shortest_path_to_goal(void);

main.c (running sequentially):


     #include <stdio.h>
     #include <string.h>
     #include "API.h"
     #include "solver.h"

// Robot state

     int robot_x = 0;
     int robot_y = 0;
     int robot_dir = NORTH;

    int main(void) {
    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));

    init_cost_map();
    flood_fill();

    printf("Phase 1: Exploring maze...\n");
    int step = 0;

    // --- Phase 1: Exploration ---
    while (1) {
        sense_and_update();
        flood_fill();
        move_to_next_cell();
        step++;

        int gx = (N - 1) / 2;
        int gy = (N - 1) / 2;
        bool at_goal = false;

        if (N % 2 == 0)
            at_goal = ((robot_x == gx || robot_x == gx + 1) &&
                       (robot_y == gy || robot_y == gy + 1));
        else
            at_goal = (robot_x == gx && robot_y == gy);

        if (at_goal) {
            API_setColor(robot_x, robot_y, 'B');
            printf("Goal reached in %d steps at (%d,%d)\n",
                   step, robot_x, robot_y);
            break;
        }

        if (step > 2000) {
            printf("Exploration exceeded 2000 steps.\n");
            break;
        }
    }

    // --- Phase 2: Return to start ---
    run_shortest_path_to_start();

    // --- Phase 3: Speed run to goal ---
    run_shortest_path_to_goal();

    printf("All phases complete.\n");
    return 0;
     }