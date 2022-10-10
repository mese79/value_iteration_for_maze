#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include <string>

#include "value_iteration.h"

using namespace std;


void GridValueIteration::run_value_iteration(float maxErr = 0.00001, int maxIterations = 1000) {
    // Performs value iteration algorithm on this maze and returns state utilities and policy.

    // 2d vector for current utilities(2d of size m_gridH, m_gridW)
    vector<vector<float>> currUtilities = initialize_utilities();
    // 2d vector of updating/next-step utilities
    vector<vector<float>> updatingUtilities = initialize_utilities();
    // policy: 2d vector of actions
    vector<vector<string>> policy(m_gridH, vector<string>(m_gridW));

    float delta = 0;    // for max difference of the current and updating utilities 
    float errCoeff = (1 - gamma) / gamma;
    int curr_it = 1;

    cout << "\nbeginning Value Iteration:\n";
    while (curr_it < maxIterations + 1) {
        cout << "\n  itertion #" << curr_it << ":\n";
        delta = 0;
        // updating each state
        for (GridState &state: m_allStates) {
            // ignore goals, pitfalls or obstacles.
            if (state.type != StateType::normal) {
                continue;
            }
            // calculate state expected utility for each action:
            // get max over actions' rewards.
            float actR = 0;
            float maxR = -9999;
            Action maxAct;
            for (auto &[key, act]: m_actions) {
                actR = calc_expected_utility(state, act, currUtilities);
                if (maxR < actR) {
                    maxR = actR;
                    maxAct = act;
                }
            }

            // cout << state.y << "," << state.x << endl;
            updatingUtilities[state.y][state.x] = maxR;
            policy[state.y][state.x] = maxAct.name;

            // float utilDiff = abs(currUtilities[state.y][state.x] - updatingUtilities[state.y][state.x]);
            // cout << "delta: " << delta << " , maxDiff: " << utilDiff << endl;
            delta = std::max(
                delta,
                abs(currUtilities[state.y][state.x] - updatingUtilities[state.y][state.x])
            );
        }

        // replace current utilities with the updated ones:
        currUtilities = updatingUtilities;

        if (delta < maxErr * errCoeff) {
            break;
        }
        curr_it++;
    }



};

void GridValueIteration::setAgentOrigin(GridState state) {
    m_agentOrigin = state;
};
void GridValueIteration::setAgentOrigin(int x, int y) {
    m_agentOrigin = {x, y};
};

//
// private methods
//
vector<GridState> GridValueIteration::generate_states(int (*goals)[2], int (*pitfalls)[2], size_t numGoals, size_t numPitfalls) {
    /*
    First: populate goals and pitfalls into vector of GridStates,
    and generate strings of positions of goals & pitfalls like : "(2,3)(5,1)"
    to make it easy to compare each state's position against goals or pitfalls.
    */
    string goals_str = "";
    for (int g = 0; g < numGoals; ++g) {
        // GridState st = {.y = goals[g][0], .x = goals[g][1]};
        // st.type = StateType::goal;
        // m_goals.push_back(st);
        goals_str += "(" + to_string(goals[g][0]) + "," + to_string(goals[g][1]) + ")";
    }
    string pitfalls_str = "";
    for (int p = 0; p < numPitfalls; ++p) {
        // GridState st = {.y = pitfalls[p][0], .x = pitfalls[p][1]};
        // st.type = StateType::pitfall;
        // m_pitfalls.push_back(st);
        pitfalls_str += "(" + to_string(pitfalls[p][0]) + "," + to_string(pitfalls[p][1]) + ")";
    }
    cout << goals_str << endl << pitfalls_str << endl;

    // generate all grid-states.
    vector<GridState> states;
    string query;
    for (int i = 0; i < m_gridH; ++i) {
        for (int j = 0; j < m_gridW; ++j) {
            GridState st = {.y = i, .x = j};
            // tile position as string: "(i,j)"
            query = "(" + to_string(i) + "," + to_string(j) + ")";
            // check for goals
            if (goals_str.find(query) != string::npos) {
                st.type = StateType::goal;
            }
            // check for pitfalls
            else if (pitfalls_str.find(query) != string::npos) {
                st.type = StateType::pitfall;
            }

            states.push_back(st);
        }
    }

    return states;
};

vector<vector<float>> GridValueIteration::initialize_utilities() {
    // create 2d vector of grid's row*cols
    vector<vector<float>> utilities(m_gridH, vector<float>(m_gridW));

    for (GridState &st: m_allStates) {
        if (st.type == StateType::goal) {
            utilities[st.y][st.x] = m_reward.goal;
        } else if(st.type == StateType::pitfall) {
            utilities[st.y][st.x] = m_reward.pitfall;
        }
    }

    return utilities;
};

float GridValueIteration::calc_expected_utility(GridState &state, Action &action, vector<vector<float>> &currUtilities) {
    /*
    Calculate expected utility of given state and action(one-step-ahead).
    U_i+1(s) = Σ_s' P(s'|s,a) (R + γ.U_i(s'))
    */
    float expectedUtility = 0;
    float R = 0;
    array<Action, 3> actions;   // for each action there are two noisy ones -> 3 actions.
    actions[0] = action;        // the intended action
    // other two unintended actions
    if (action.name == ACT_UP or action.name == ACT_DOWN) {
        actions[1] = this->m_actions[ACT_LEFT];
        actions[2] = this->m_actions[ACT_RIGHT];
    } else {
        actions[1] = this->m_actions[ACT_UP];
        actions[2] = this->m_actions[ACT_DOWN];
    }
    // compute the expected utility
    for (int i = 0; i < 3; ++i) {
        // get the next state given the action
        GridState nextState = apply_action(state, actions[i]);
        // grid-bound checking:
        if(not is_inside_maze(nextState)) {
            nextState = state;
            R += m_reward.obstacle;     // penalty for hitting wall or obstacle
        }
        R += m_reward.move;             // cost of movement

        expectedUtility += m_actionProbs[i] * (R + gamma * currUtilities[nextState.y][nextState.x]);
    }

    return expectedUtility;
}

GridState GridValueIteration::apply_action(GridState &state, Action &action) {
    GridState newState;
    newState.y = state.y + action.deltaY;
    newState.x = state.x + action.deltaX;
    return newState;
};

bool GridValueIteration::is_inside_maze(GridState &state) {
    return (state.y >= 0 and state.y < m_gridH) and (state.x >= 0 and state.x < m_gridW);
};


// test the class
int main() {
    //! NOTE: This implementation is ROW-BASED, so each state's address is like: (y, x) .
    cout << "value iteration main" << endl;
    int rows = 4;
    int cols = 6;
    int goals[1][2] = { {2, 2} };
    int pitfalls[4][2] = {
        {3, 0}, {1, 1}, {2, 3}, {2, 4}
    };
    float action_prob = 0.8;
    GridValueIteration maze = GridValueIteration(rows, cols, action_prob, goals, pitfalls);
    maze.run_value_iteration(0.00001, 100);
}