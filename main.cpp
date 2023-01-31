#include <iostream>
#include <iterator>
#include <fstream>
#include <string>
#include <vector>

#include "jsoncpp/json/json.h"
#include "value_iteration.h"
#include "utils.h"

using namespace std;


int main() {
    int rows = 4;
    int cols = 6;
    //! NOTE: This implementation is ROW-BASED, so each state's address is like: (y, x) .
    int goals[1][2] = { {2, 2} };
    int pitfalls[4][2] = {
        {3, 0}, {1, 1}, {2, 3}, {2, 4}
    };
    double action_prob = 0.8;
    bool printLog = true;

    // set the rewards/penalties
    Reward rewards = {
        .goal = 1,
        .pitfall = -500,
        .obstacle = -0.1,     // hitting wall/obstacle penalty
        .move = -0            // penalty for each move
    };

    GridValueIteration maze = GridValueIteration(rows, cols, action_prob, goals, pitfalls, printLog);
    maze.set_reward(rewards);
    maze.gamma = 0.9;
    // run the algorothm
    vector<vector<string>> policy = maze.run_value_iteration(0.000001, 1000);
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

    // save optimized policy in actions.txt
    std::ofstream output_file("./actions.txt");
    std::ostream_iterator<std::string> output_iterator(output_file, "\n");
    std::copy(actions.begin(), actions.end(), output_iterator);
    std::cout << "Result wrote on file actions.txt" << endl;

    return 0;
}
