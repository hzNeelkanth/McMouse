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

