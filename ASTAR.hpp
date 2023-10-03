#pragma once
#include <queue>
using namespace std;

class ASTAR
{
    double*	map;
    int collision_thresh;
    int x_size;
    int y_size;
    int robotposeX;
    int robotposeY;
    int target_steps;
    double* target_traj;
    int targetposeX;
    int targetposeY;
    int curr_time;
    double* action_ptr;

    struct node
    {
        int parent = -1;
        int g = INT_MAX;
    };

    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openList;   //f-value, cell index (sorted in increasing order of f-value)

    unordered_map<int, bool> closedList; // key: index, value: whether or not node has been expanded
    unordered_map<int, node> cellInfo; // key: index, value: node

    int successors[2][8] = {{-1, 0, 1, -1, 1, -1, 0, 1},
                            {-1, -1, -1, 0, 0, 1, 1, 1}};


    public:

    // goal position (last position on object trajectory)
    int goalposeX = 1;
    int goalposeY = 1;

    // construct path to goal in reverse (start at goal and follow parents all the way to robot position)
    stack <int> returnPath;

    ASTAR(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
    )
    {
        this->map = map;
        this->collision_thresh = collision_thresh;
        this->x_size = x_size;
        this->y_size = y_size;
        this->robotposeX = robotposeX;
        this->robotposeY = robotposeY;
        this->target_steps = target_steps;
        this->target_traj = target_traj;
        this->targetposeX = targetposeX;
        this->targetposeY = targetposeY;
        this->curr_time = curr_time;
        this->action_ptr = action_ptr;
    }

    int XYtoIndex(int x, int y)
    {
        return (x_size*(y-1)) + (x-1); 
    }

    pair<int, int> indexToXY(int index)
    {
        return make_pair((index % x_size) + 1, (index / x_size) + 1);
    }

    // diagonal distance
    int calculate_heuristic(int x_pos, int y_pos)
    {
        return max(abs(x_pos-goalposeX), abs(y_pos - goalposeY));
    }

    void initStartCell()
    {
        int starting_pos = XYtoIndex(robotposeX, robotposeY);
        cellInfo[starting_pos].g = 0;
        int h_value = calculate_heuristic(robotposeX, robotposeY);
        this->openList.push(make_pair(h_value, starting_pos));
    }

    // last position on trajectory
    void findGoal()
    {
        goalposeX = (int) target_traj[target_steps-1];
        goalposeY = (int) target_traj[target_steps-1+target_steps];
    }

    void backTrack();
    int sIsValid(int);
    int sIndex(int, int);
    void computePath();
};

