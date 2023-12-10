#include "cubicgen.hpp"

#include <chrono>

int main() {
    using namespace std::chrono_literals;

    shape_sequence seq{};

    cuboid cub_obj{ { -0.3, -0.3, -0.3 }, { 0.4, 0.4, 0.4 } };
    seq.add_keyframe(0s, cub_obj);
    seq.add_keyframe(1s, cub_obj.move({ 0.6, 0, 0 }));
    seq.add_keyframe(2s, cub_obj.move({ 0, 0.6, 0 }));
    seq.add_keyframe(3s, cub_obj.move({ 0, 0, 0.6 }));
    seq.add_keyframe(4s, cub_obj.move({ -0.6, 0, 0 }));
    seq.add_keyframe(5s, cub_obj.move({ 0, -0.6, 0 }));
    seq.add_keyframe(6s, cub_obj.move({ 0, 0, -0.6 }));

    const auto& c = seq.rasterize<5>(4, 24ms, 6s);  // suppose 5x5x5 cube
    c.write("test.cbi");

    return 0;
}
