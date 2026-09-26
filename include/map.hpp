#pragma once

#include <iostream>
#include <vector>
#include "public_data.hpp"
class map{
    private:
        std::vector<std::vector<int>> grid_map;
        int rows;
        int cols;
        GridPoint start;
        GridPoint goal;
    public:
        map(int rows,int cols);
        void set_start(const GridPoint & st);
        void set_goal(const GridPoint & end);
        bool set_obstacle(const GridPoint & point);
        int get_rows(void) const;
        int get_cols(void) const;
        const GridPoint& get_start(void) const;
        const GridPoint& get_goal(void) const;
        bool point_isvalid(GridPoint cur_point) const;
        bool is_obstacle(GridPoint cur_point) const;
};
