#include "cubicgen.hpp"

int main() {
    shape_sequence seq{};

    cuboid cub_obj{ { -0.3, -0.3, -0.3 }, { 0.4, 0.4, 0.4 } };
    seq.add_keyframe(0, cub_obj);
    seq.add_keyframe(1000, cub_obj.move({ 0.6, 0, 0 }));
    seq.add_keyframe(2000, cub_obj.move({ 0, 0.6, 0 }));
    seq.add_keyframe(3000, cub_obj.move({ 0, 0, 0.6 }));
    seq.add_keyframe(4000, cub_obj.move({ -0.6, 0, 0 }));
    seq.add_keyframe(5000, cub_obj.move({ 0, -0.6, 0 }));
    seq.add_keyframe(6000, cub_obj.move({ 0, 0, -0.6 }));

    const auto& c = seq.rasterize<5>(4, 24, 6000);  // suppose 5x5x5 cube
    c.write("test.cbi");

    return 0;
}
