#include <stdio.h>

#include "API.h"

void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    log("Running...");
    API_setColor(0, 0, 'G');
    API_setText(0, 0, "abc");
    while (1) {
        if (!API_wallLeft()) {
            API_turnLeft();
        }
        while (API_wallFront()) {
            API_turnRight();
        }
        API_moveForward();
    }
}

#include <stdio.h>
#include <stdbool.h>
#include "solver.h"

// Example maze definition (walls hardcoded for simulation)
// 1 = wall, 0 = no wall
void setup_test_maze(void) {
    // Clear all walls
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
            for (int d = 0; d < 4; d++) {
                walls[x][y][d] = false;
            }
        }
    }

    // Add some example walls (you can customize)
    // Outer borders
    for (int i = 0; i < N; i++) {
        walls[i][0][SOUTH] = true;
        walls[i][N-1][NORTH] = true;
        walls[0][i][WEST] = true;
        walls[N-1][i][EAST] = true;
    }

    // Add a simple internal wall for testing
    for (int y = 0; y < N/2; y++) {
        walls[4][y][EAST] = true;
        walls[5][y][WEST] = true;
    }
}




int main(void) {
    printf("=== Flood-Fill Maze Solver Simulation ===\n");

    // Initialize the maze and solver
    setup_test_maze();
    init_cost_map();
    flood_fill();

    printf("\nInitial cost map:\n");
    print_cost_map();

    // Simulate robot starting at (0,0)
    robot_x = 0;
    robot_y = 0;
    robot_dir = NORTH;

    // Run until the robot reaches the goal
    int step = 0;
    while (!(robot_x == goal_x && robot_y == goal_y)) {
        printf("\nStep %d:\n", step);
        move_robot();
        flood_fill();  // Recompute distances if walls are updated dynamically
        step++;

        if (step > 100) {
            printf("Too many steps, aborting.\n");
            break;
        }
    }

    if (robot_x == goal_x && robot_y == goal_y)
        printf("\nGoal reached at (%d,%d)!\n", goal_x, goal_y);
    else
        printf("\nGoal not reached.\n");

    return 0;
}