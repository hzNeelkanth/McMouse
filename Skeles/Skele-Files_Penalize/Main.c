#include <stdio.h>
#include <string.h> // required for memset
#include "API.h"
#include "solver.h"

// Robot state
int robot_x = 0;
int robot_y = 0;
int robot_dir = NORTH;

int main(void) {
    // Initialize robot position and maze
    robot_x = 0;
    robot_y = 0;
    robot_dir = NORTH;

    memset(walls, 0, sizeof(walls));
    memset(visited, 0, sizeof(visited));

    // Initialize flood-fill costs
    init_cost_map();
    flood_fill();

    // Main loop
    while (1) {
        // Sense walls and update maze
        sense_and_update();

        // Move to next cell
        move_to_next_cell();

        // Recalculate flood-fill after every move
        flood_fill();

        // Stop if goal reached (center 2x2)
        if ((robot_x == N/2 || robot_x == N/2-1) &&
            (robot_y == N/2 || robot_y == N/2-1)) {
            API_setColor(robot_x, robot_y, 'B'); // mark goal
            break;
        }
    }

    return 0;
}
