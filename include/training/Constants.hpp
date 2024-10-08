#include <vector>
#include <string>

namespace Constants {
    struct turtle_info {
        std::string name;
        float x_pos;
        float y_pos;
        float rad;
    };

    extern const std::vector<turtle_info> turtle_bio;
}