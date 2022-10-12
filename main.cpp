#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>

#include "jsoncpp/json/json.h"
#include "value_iteration.h"
#include "utils.h"

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

    // set the rewards/penalties
    // default is: 1 for goals, and -1 for pitfalls
    Reward rewards = {
        .goal = 1,
        .pitfall = -1,
        .obstacle = -0,     // hitting wall/obstacle penalty
        .move = -0          // penalty for each move
    };

    GridValueIteration maze = GridValueIteration(rows, cols, action_prob, goals, pitfalls, printLog);
    maze.set_reward(rewards);
    // run the algorothm
    vector<vector<string>> policy = maze.run_value_iteration(0.00001, 1000);
    // generate actions output 
    vector<string> actions;
    actions = policy_to_string(policy);

    // save maze properties to json file(could be used for show.py)
    Json::Value config;
    config["rows"] = rows;
    config["cols"] = cols;
    int num_goals = sizeof(goals) / sizeof(goals[0]);
    for (int i = 0; i < num_goals; i++) {
        config["goals"][to_string(i)]["y"] = goals[i][0];
        config["goals"][to_string(i)]["x"] = goals[i][1];
    }
    int num_pits = sizeof(pitfalls) / sizeof(pitfalls[0]);
    for (int i = 0; i < num_pits; i++) {
        config["pitfalls"][to_string(i)]["y"] = pitfalls[i][0];
        config["pitfalls"][to_string(i)]["x"] = pitfalls[i][1];
    }
    // save config to file
    ofstream config_file;
    config_file.open("config.json");
    config_file << config;
    config_file.close();

    // DO NOT CHANGE THE FOLLOWING CODE

    std::ofstream output_file("./actions.txt");
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(actions.begin(), actions.end(), output_iterator);
    std::cout << "Result wrote on file actions.txt" << endl;

    return 0;
}
