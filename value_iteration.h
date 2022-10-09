#ifndef PLAYER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define PLAYER_H

#include <iostream>
#include <array>
#include <vector>
#include <string>

using namespace std;


const string ACT_UP = "Up";
const string ACT_DOWN = "Down";
const string ACT_LEFT = "Left";
const string ACT_RIGHT = "Right";
// enum class ActionEnum {up = 1, down, left, right};

enum class StateType {normal = 1, goal, pitfall, obstacle};

struct GridState {
    int x;
    int y;
    StateType type = StateType::normal;
};

struct Action {
    string name;
    int deltaY;
    int deltaX;
    Action(){}
    Action(string name, int deltaRow, int deltaCol) {}
};

struct Reward {
    float goal = 1;
    float pitfall = -1;
    float obstacle = 0;
    float move = 0;
};

class GridValueIteration {
    public:
        float gamma;
        // constructor with template should be implemented in thev header file.
        template <size_t gRows, size_t pRows>
        GridValueIteration(
            int gridH, int gridW, float actionProb,
            int (&goals)[gRows][2], int (&pitfalls)[pRows][2]
        ) {
            m_gridH = gridH;
            m_gridW = gridW;
            m_actionProb = actionProb;
            m_actionNoiseProb = (1 - actionProb) / 2;
            m_agentOrigin = {0, 0};  // default start state

            // initialize all actions (row first)
            m_actions[0] = Action("Up", 1, 0);
            m_actions[1] = Action("Down", -1, 0);
            m_actions[2] = Action("Right", 0, 1);
            m_actions[3] = Action("Left", 0, -1);

            gamma = 0.9;
            m_allStates = generate_states(goals, pitfalls, gRows, pRows);
        };

        void run_value_iteration(float max_err, int max_iterations);
        void setAgentOrigin(GridState state);
        void setAgentOrigin(int x, int y);

    private:
        int m_gridW;
        int m_gridH;
        vector<GridState> m_allStates;
        // vector<GridState> m_goals;
        // vector<GridState> m_pitfalls;
        Reward m_reward;
        float m_actionProb;
        float m_actionNoiseProb;
        GridState m_agentOrigin;
        Action m_actions[4] = {};
        GridState m_currState;

        vector<GridState> generate_states(int (*goals)[2], int (*pitfalls)[2], size_t gRows, size_t pRows);
        vector<vector<float>> initialize_utilities();

        template <size_t rows, size_t cols>
        float calc_expected_utility(GridState state, Action action, float (&currUtilities)[rows][cols]);
        GridState apply_action(GridState state, Action action);
};

#endif