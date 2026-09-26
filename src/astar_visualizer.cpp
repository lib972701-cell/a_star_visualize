#include "astar_visualizer.hpp"

#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

AstarVisualizer::AstarVisualizer(int cell_size, std::string window_name)
    : cell_size(std::max(cell_size, 12)),
      window_name(std::move(window_name))
{
    cv::namedWindow(this->window_name, cv::WINDOW_AUTOSIZE);
}

cv::Rect AstarVisualizer::cell_rect(int row, int col) const
{
    return {col * cell_size, row * cell_size, cell_size, cell_size};
}

bool AstarVisualizer::show(
    const map &grid,
    const std::vector<std::vector<int>> &visited,
    GridPoint current,
    const std::vector<GridPoint> &shortest_path,
    int delay_ms)
{
    const int legend_height = 42;
    cv::Mat image(
        grid.get_rows() * cell_size + legend_height,
        grid.get_cols() * cell_size,
        CV_8UC3,
        cv::Scalar(245, 245, 245)
    );

    const cv::Scalar empty_color(245, 245, 245);
    const cv::Scalar obstacle_color(35, 35, 35);
    const cv::Scalar visited_color(255, 210, 120);
    const cv::Scalar current_color(0, 140, 255);
    const cv::Scalar path_color(0, 255, 255);
    const cv::Scalar start_color(70, 200, 70);
    const cv::Scalar goal_color(70, 70, 230);

    for(int row = 0; row < grid.get_rows(); ++row)
    {
        for(int col = 0; col < grid.get_cols(); ++col)
        {
            cv::Scalar color = empty_color;
            if(grid.is_obstacle({row, col}))
            {
                color = obstacle_color;
            }
            else if(row < static_cast<int>(visited.size()) &&
                    col < static_cast<int>(visited[row].size()) &&
                    visited[row][col])
            {
                color = visited_color;
            }
            cv::rectangle(image, cell_rect(row, col), color, cv::FILLED);
        }
    }

    for(const GridPoint &point : shortest_path)
    {
        cv::rectangle(image, cell_rect(point.row, point.col), path_color, cv::FILLED);
    }

    if(current.row >= 0 && current.col >= 0 && shortest_path.empty())
    {
        cv::rectangle(image, cell_rect(current.row, current.col), current_color, cv::FILLED);
    }

    GridPoint start = grid.get_start();
    GridPoint goal = grid.get_goal();
    cv::rectangle(image, cell_rect(start.row, start.col), start_color, cv::FILLED);
    cv::rectangle(image, cell_rect(goal.row, goal.col), goal_color, cv::FILLED);

    for(int row = 0; row <= grid.get_rows(); ++row)
    {
        int y = row * cell_size;
        cv::line(image, {0, y}, {grid.get_cols() * cell_size, y}, {100, 100, 100}, 1);
    }
    for(int col = 0; col <= grid.get_cols(); ++col)
    {
        int x = col * cell_size;
        cv::line(image, {x, 0}, {x, grid.get_rows() * cell_size}, {100, 100, 100}, 1);
    }

    auto draw_label = [&](GridPoint point, const std::string &text) {
        int baseline = 0;
        cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
        cv::Point origin(
            point.col * cell_size + (cell_size - size.width) / 2,
            point.row * cell_size + (cell_size + size.height) / 2
        );
        cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    {255, 255, 255}, 2, cv::LINE_AA);
    };
    draw_label(start, "S");
    draw_label(goal, "G");

    int text_y = grid.get_rows() * cell_size + 28;
    std::string legend = shortest_path.empty()
        ? "Blue: expanded   Orange: current   Black: obstacle   Esc: quit"
        : "Yellow: shortest path   S: start   G: goal   Press any key to close";
    cv::putText(image, legend, {8, text_y}, cv::FONT_HERSHEY_SIMPLEX,
                0.43, {40, 40, 40}, 1, cv::LINE_AA);

    cv::imshow(window_name, image);
    int key = cv::waitKey(delay_ms);
    return key != 27;
}
