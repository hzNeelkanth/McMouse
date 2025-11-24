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

printf("Starting micromouse flood-fill solver...\n");

    int step = 0;
    while (1) {
        sense_and_update();   // detect walls and mirror
        flood_fill();         // recompute cost map
        move_to_next_cell();  // move toward goal

        step++;

        // Stop at center (1×1 or 2×2)
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

        if (step > 1000) { // safety break
            printf("Exceeded 1000 steps without reaching goal.\n");
            break;
        }
    }

    return 0;
}