#include "cubicgen.hpp"
#include <chrono>

int main() {
    using namespace std::chrono_literals;

   constexpr auto pi = 3.14159265358979323846;

    shape_sequence seq{};

    cuboid cube_obj{{ -0.4, -0.4, 0 }, { 0.2, 0.2, 1.0 }};
    seq.add_keyframe(0s, cube_obj);
    seq.add_keyframe(250ms, cube_obj.rotate(quaternion::around(vec3::y_axis(), pi / 2), { -0.4, -0.4, -0.4 }));
    seq.add_keyframe(500ms, cube_obj.rotate(quaternion::around(vec3::z_axis(), -pi / 2), { 0.4, -0.4, -0.4 }));
    seq.add_keyframe(750ms, cube_obj.rotate(quaternion::around(vec3::x_axis(), -pi / 2), { 0.4, 0.4, -0.4 }));
    seq.add_keyframe(1000ms, cube_obj.rotate(quaternion::around(vec3::y_axis(), pi / 2), { 0.4, 0.4, 0.4 }));
    seq.add_keyframe(1250ms, cube_obj.rotate(quaternion::around(vec3::z_axis(), -pi / 2), { -0.4, 0.4, 0.4 }));
    seq.add_keyframe(1500ms, cube_obj.rotate(quaternion::around(vec3::x_axis(), -pi / 2), { -0.4, -0.4, 0.4 }));

    const auto& c = seq.rasterize<5>(4, 25ms, 1500ms);  // suppose 5x5x5 cube
    c.write("test.cbi");

    return 0;
}
