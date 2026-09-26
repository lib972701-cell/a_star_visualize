#include <algorithm>
#include <cstdlib>
#include <limits>
#include <queue>
#include <vector>
#include "a_star.hpp"

Astar::Astar(map g_map):Grid_map(g_map),
dir{{0,1,0,-1},{1,0,-1,0}}
{}

void Astar::initialize_Astar()
{
    int col = Grid_map.get_cols();
    int row = Grid_map.get_rows();
    isvisited.assign(row, std::vector<int>(col, 0));
    g_score.assign(row, std::vector<int>(col, std::numeric_limits<int>::max()));
    path.assign(row, std::vector<GridPoint>(col, {-1,-1}));
    shortest_path.clear();
    current_point = {-1, -1};
    while(!openLists.empty())
    {
        openLists.pop();
    }

    GridPoint start = Grid_map.get_start();
    node start_node;
    start_node.point = start;
    start_node.g = 0;
    start_node.h = Heuristic(start);
    start_node.f = start_node.g + start_node.h;
    g_score[start.row][start.col] = 0;
    openLists.push(start_node);
}

int Astar::Heuristic(const GridPoint &point)
{
    GridPoint goal = Grid_map.get_goal();
    return std::abs(goal.row - point.row) + std::abs(goal.col - point.col);
}

void Astar::build_path()
{
    GridPoint start,cur;
    start = Grid_map.get_start();
    cur = Grid_map.get_goal();
    shortest_path.clear();
    while((cur.row != start.row) || (cur.col != start.col))
    {
        shortest_path.push_back(cur);
        cur = path[cur.row][cur.col];
        if(cur.row == -1 || cur.col == -1)
        {
            shortest_path.clear();
            return;
        }
    }
    shortest_path.push_back(start);
    std::reverse(shortest_path.begin(), shortest_path.end());
}

const std::vector<GridPoint>& Astar::get_path() const
{
    return shortest_path;
}

const std::vector<std::vector<int>>& Astar::get_visited() const
{
    return isvisited;
}

GridPoint Astar::get_current_point() const
{
    return current_point;
}

state_of_search Astar::step()
{
    while(!openLists.empty())
    {
        node cur_node = openLists.top();
        openLists.pop();

        int cur_row = cur_node.point.row;
        int cur_col = cur_node.point.col;
        if(isvisited[cur_row][cur_col])
        {
            continue;
        }
        isvisited[cur_row][cur_col] = 1;
        current_point = cur_node.point;

        GridPoint goal = Grid_map.get_goal();
        if(cur_row == goal.row && cur_col == goal.col)
        {
            build_path();
            return state_of_search::success;
        }

        for(int i=0;i<4;i++)
        {
            GridPoint next_point{
                cur_row + dir[0][i],
                cur_col + dir[1][i]
            };

            if(!Grid_map.point_isvalid(next_point) || Grid_map.is_obstacle(next_point))
            {
                continue;
            }
            if(isvisited[next_point.row][next_point.col])
            {
                continue;
            }

            int new_g = cur_node.g + 1;
            if(new_g < g_score[next_point.row][next_point.col])
            {
                g_score[next_point.row][next_point.col] = new_g;
                path[next_point.row][next_point.col] = cur_node.point;

                node next_node;
                next_node.point = next_point;
                next_node.g = new_g;
                next_node.h = Heuristic(next_point);
                next_node.f = next_node.g + next_node.h;
                openLists.push(next_node);
            }
        }
        return state_of_search::searching;
    }

    return state_of_search::falure;
}
