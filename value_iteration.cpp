#include <iostream>
#include <array>
#include <vector>
#include <string>

#include "value_iteration.h"

using namespace std;


void GridValueIteration::run_value_iteration(float max_err = 0.00001, int max_iterations = 100) {
    // Performs value iteration algorithm on this maze and returns state utilities and policy.
    float err_coeff = (1 - gamma) / gamma;
    // utilities at each iteration (2d of size m_gridH, m_gridW)
    vector<vector<float>> curr_utilities = initialize_utilities();
    // policy: 2d vector of actions
    vector<vector<Action>> policy(m_gridH, vector<Action>(m_gridW));
    // discounted_rewards: 2d vector of updating rewards
    vector<vector<float>> discounted_rewards(curr_utilities.begin(), curr_utilities.end());

    // current_utilities = self._get_utilities_at_0()
    // updating_utilities = current_utilities.copy()

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
// , int gRows, int pRows
vector<GridState> GridValueIteration::generate_states(int (*goals)[2], int (*pitfalls)[2], size_t gRows, size_t pRows) {
    /*
    First: populate goals and pitfalls into vector of GridStates,
    and generate strings of positions of goals & pitfalls like : "(2,3)(5,1)"
    to make it easy to compare each state's position against goals or pitfalls.
    */
    string goals_str = "";
    for(int g = 0; g < gRows; ++g) {
        // GridState st = {goals[g][0], goals[g][1]};
        // st.type = StateType::goal;
        // m_goals.push_back(st);
        goals_str += "(" + to_string(goals[g][0]) + "," + to_string(goals[g][1]) + ")";
    }
    string pitfalls_str = "";
    for(int p = 0; p < pRows; ++p) {
        // GridState st = {pitfalls[p][0], pitfalls[p][1]};
        // st.type = StateType::pitfall;
        // m_pitfalls.push_back(st);
        pitfalls_str += "(" + to_string(pitfalls[p][0]) + "," + to_string(pitfalls[p][1]) + ")";
    }
    // cout << goals_str << endl << pitfalls_str << endl;

    // generate all grid-states.
    vector<GridState> states;
    string query;
    for(int i = 0; i < m_gridH; ++i) {
        for(int j = 0; j < m_gridW; ++j) {
            GridState st = {i, j};
            // tile position as string: "(i,j)"
            query = "(" + to_string(i) + "," + to_string(j) + ")";
            // check for goals
            if (goals_str.find(query) != string::npos) {
                st.type = StateType::goal;
            }
            // check for pitfalls
            if (pitfalls_str.find(query) != string::npos) {
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

    for (GridState st: m_allStates) {
        if(st.type == StateType::goal) {
            utilities[st.y][st.x] = m_reward.goal;
        } else if(st.type == StateType::pitfall) {
            utilities[st.y][st.x] = m_reward.pitfall;
        }
    }

    return utilities;
};

template <size_t rows, size_t cols>
float GridValueIteration::calc_expected_utility(GridState state, Action action, float (&currUtilities)[rows][cols]) {
    /*
    Calculate expected utility of given state and action(one-step-ahead).
    U_i+1(s) = Σ_s' P(s'|s,a) (R + γ.U_i(s'))
    */
    float state_exp_utility = 0;
    float R = 0;

    for(Action act: m_actions) {
        GridState nextState = apply_action(state, action);
        R += m_reward.move;  // cost of movement
    }

    return state_exp_utility;
}

GridState GridValueIteration::apply_action(GridState state, Action action) {
    GridState newState;
    newState.y = state.y + action.deltaY;
    newState.x = state.x + action.deltaX;
    return newState;
};
