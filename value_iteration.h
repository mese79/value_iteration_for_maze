#ifndef VALUE_ITERATION_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define VALUE_ITERATION_H

#include <iostream>
#include <array>
#include <vector>
#include <map>
#include <string>

using namespace std;


const string ACT_UP = "Up";
const string ACT_DOWN = "Down";
const string ACT_LEFT = "Left";
const string ACT_RIGHT = "Right";
const string GOAL_SYM = "*G*";
const string PITFALL_SYM = "!P!";

enum class StateType {normal = 1, goal, pitfall, obstacle};

struct GridState {
    int y = -1;
    int x = -1;
    StateType type = StateType::normal;
    // == operator
    bool operator==(const GridState &other) {
        return (y == other.y) and (x == other.x) and (type == other.type);
    };
};

struct Action {
    string name;
    int deltaY;
    int deltaX;
};

struct Reward {
    float goal = 1;
    float pitfall = -1;
    float obstacle = 0;
    float move = 0;
};

class GridValueIteration {
    private:
        int m_gridW;
        int m_gridH;
        vector<GridState> m_allStates;
        // vector<GridState> m_goals;
        // vector<GridState> m_pitfalls;
        Reward m_reward;                        // hold different rewards
        float m_actionProbs[3];                 // actions' probabilities: intentended + two noisy ones
        map<string, Action> m_actions;          // all possible actions
        bool m_printLog;                        // log the process output into stdout

        vector<GridState> generate_states(int (*goals)[2], int (*pitfalls)[2], size_t numGoals, size_t numPitfalls);
        
        vector<vector<float>> initialize_utilities();

        float calc_expected_utility(GridState &state, Action &action, vector<vector<float>> &currUtilities);

        GridState apply_action(GridState &state, Action &action);

        bool is_inside_maze(GridState &state);

        void set_policy_fixed_symbols(vector<vector<string>> &policy);

        void replace_symbol_with_action(vector<vector<string>> &policy);

    public:
        float gamma;
        map<const string, string> actionSymbol;
        
        // constructor with template should be implemented in the header file.
        template <size_t gRows, size_t pRows>
        GridValueIteration(
            int gridH, int gridW, float actionProb,
            int (&goals)[gRows][2], int (&pitfalls)[pRows][2],
            bool printLog = false
        ) {
            //! NOTE: This implementation is ROW-BASED, so each state's address is like: (y, x) .
            m_gridH = gridH;
            m_gridW = gridW;

            // actions' probabilities: intentended + two noisy ones
            m_actionProbs[0] = actionProb;
            m_actionProbs[1] = (1 - actionProb) / 2;
            m_actionProbs[2] = m_actionProbs[1];

            // initialize all actions (delta: row, col)
            m_actions[ACT_UP] = {.name = "Up", .deltaY = 1, .deltaX = 0};
            m_actions[ACT_DOWN] = {.name = "Down", .deltaY = -1, .deltaX = 0};
            m_actions[ACT_RIGHT] = {.name = "Right", .deltaY = 0, .deltaX = 1};
            m_actions[ACT_LEFT] = {.name = "Left", .deltaY = 0, .deltaX = -1};

            actionSymbol[ACT_UP] = "^";
            actionSymbol[ACT_DOWN] = "v";
            actionSymbol[ACT_RIGHT] = ">";
            actionSymbol[ACT_LEFT] = "<";

            m_printLog = printLog;
            gamma = 0.9;
            // generate all states
            m_allStates = generate_states(goals, pitfalls, gRows, pRows);
        };

        vector<vector<string>> run_value_iteration(float maxErr, int maxIterations);
        void set_reward(Reward &reward);
};


#endif