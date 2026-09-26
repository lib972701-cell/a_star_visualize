#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "map.hpp"

class AstarVisualizer
{
public:
    AstarVisualizer(int cell_size = 48,
                    std::string window_name = "A* Search Visualization");

    // 返回 false 表示用户按下 Esc，希望终止演示。
    bool show(const map &grid,
              const std::vector<std::vector<int>> &visited,
              GridPoint current,
              const std::vector<GridPoint> &shortest_path = {},
              int delay_ms = 120);

private:
    int cell_size;
    std::string window_name;

    cv::Rect cell_rect(int row, int col) const;
};
