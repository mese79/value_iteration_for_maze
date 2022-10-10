#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>

#include "value_iteration.h"

using namespace std;

/*
 * Replace the values inside the vector "actions" with the one of your solution.
 * The index of the vector increases from the bottom left to top right.
 * The value of the vector indicates the action to perform at the corresponding state.
 * You can show on screen your solution running the python script "show.py" in the build folder.
 */

int main() {
    int rows = 4;
    int cols = 6;
    //! NOTE: This implementation is ROW-BASED, so each state's address is like: (y, x) .
    int goals[1][2] = { {2, 2} };
    int pitfalls[4][2] = {
        {3, 0}, {1, 1}, {2, 3}, {2, 4}
    };
    float action_prob = 0.8;
    bool printLog = true;
    GridValueIteration maze = GridValueIteration(rows, cols, action_prob, goals, pitfalls, printLog);
    maze.run_value_iteration(0.0001, 100);


    vector<string> actions = {
        "up", "down", "left", "right",
        "up", "down", "left", "right",
        "up", "down", "left", "right",
        "up", "down", "left", "right",
        "up", "down", "left", "right",
        "up", "down", "left", "right"
    };



    // DO NOT CHANGE THE FOLLOWING CODE

    std::ofstream output_file("./actions.txt");
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(actions.begin(), actions.end(), output_iterator);
    std::cout << "Result wrote on file actions.txt" << endl;

    return 0;
}
