#pragma once

#include "map.hpp"
#include <queue>
#include <vector>

class Astar{
    private:
        struct node{
            GridPoint point;
            int h = 0;
            int g = 0;
            int f = 0;
            bool operator < (const node & k) const{  // 重载运算符， 从小到大排序
                    return k.f < f;}
        };
        map Grid_map;
        std::vector<std::vector<int>> isvisited;
        std::vector<std::vector<int>> g_score;
        std::vector<std::vector<GridPoint>> path;
        std::vector<GridPoint> shortest_path;
        std::priority_queue<node> openLists;
        int dir[2][4];
        GridPoint current_point{-1, -1};

        int Heuristic(const GridPoint &point);
        void build_path();
    public:
        Astar(map g_map);
        void initialize_Astar(void);
        state_of_search step(void);
        const std::vector<GridPoint>& get_path() const;
        const std::vector<std::vector<int>>& get_visited() const;
        GridPoint get_current_point() const;
};
