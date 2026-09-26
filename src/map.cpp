#include "map.hpp"
#include <vector>

map::map(int row,int col)
    :grid_map(row,std::vector<int>(col,0)), rows(row), cols(col)
{}

void map::set_start(const GridPoint & st)
{
    start = st;
}

void map::set_goal(const GridPoint & end)
{
    goal = end;
}

bool map::set_obstacle(const GridPoint & point)
{
    if(!point_isvalid(point))
    {
        return false;
    }
    grid_map[point.row][point.col] = 1;
    return true;
}

int map::get_rows() const
{
    return rows;
}

int map::get_cols() const
{
    return cols;
}

const GridPoint& map:: get_start() const
{
    return start;
}

const GridPoint& map:: get_goal() const
{
    return goal;
}

bool map::point_isvalid(GridPoint cur_point) const
{
    if(cur_point.col < 0 || cur_point.row < 0 || cur_point.col >= cols || cur_point.row >= rows)
    {
        return false;
    }
    else{return true;}
}

bool map::is_obstacle(GridPoint cur_point) const
{
    if(grid_map[cur_point.row][cur_point.col] == 1)
    {return true;}
    else{return false;}
}
