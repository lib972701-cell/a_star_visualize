#include <string>
#include <iostream>
#include <vector>
#include "a_star.hpp"
#include "astar_visualizer.hpp"

int main(int argc, char *argv[])
{
    bool auto_close = argc > 1 && std::string(argv[1]) == "--auto-close";
    map grid(8, 10);
    grid.set_start({0, 0});
    grid.set_goal({7, 9});

    // 竖直墙在第 4 列，只在第 5 行留一个缺口，迫使路径绕行。
    for(int row = 0; row < grid.get_rows(); ++row)
    {
        if(row != 5)
        {
            grid.set_obstacle({row, 4});
        }
    }

    // 再增加一小段水平障碍，用于测试转向。
    grid.set_obstacle({3, 1});
    grid.set_obstacle({3, 2});
    grid.set_obstacle({3, 3});

    Astar astar(grid);
    astar.initialize_Astar();
    AstarVisualizer visualizer;

    visualizer.show(
        grid,
        astar.get_visited(),
        astar.get_current_point(),
        {},
        300
    );

    state_of_search state = searching;
    while(state == searching)
    {
        state = astar.step();
        if(!visualizer.show(
               grid,
               astar.get_visited(),
               astar.get_current_point(),
               {},
               120))
        {
            return 0;
        }
    }

    if(state == falure)
    {
        std::cout << "没有找到可行路径。\n";
        visualizer.show(grid, astar.get_visited(), astar.get_current_point(), {},
                        auto_close ? 500 : 0);
        return 1;
    }

    const std::vector<GridPoint> &path = astar.get_path();
    std::cout << "找到最短路径，共 " << path.size() - 1 << " 步：\n";

    for(const GridPoint &point : path)
    {
        std::cout << '(' << point.row << ',' << point.col << ") ";
    }
    std::cout << '\n';
    visualizer.show(grid, astar.get_visited(), astar.get_current_point(), path,
                    auto_close ? 500 : 0);
    return 0;
}
