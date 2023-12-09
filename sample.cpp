#include "cubicgen.hpp"

int main() {
    shape_sequence seq{};

    

    /*
    cuboid cube_obj{ vec3{ 0., 0., 0. }, vec3{ 1., 1., 1. } };

    seq.add_keyframe(0, cube_obj);
    seq.add_keyframe(1000, cube_obj.scale( vec3{ .5, .5, .5 } ));
    seq.add_keyframe(2000, cube_obj.move( vec3{ 0.2, 0, 0 } ));
    seq.add_keyframe(3000, cube_obj.erase());
    */

    ellipsoid ell_obj{ vec3{ 0., 0., 0. }, vec3{ .5, .5, .5 } };

    seq.add_keyframe(0, ell_obj);
    seq.add_keyframe(1000, ell_obj.scale({ 0.5, 0.5, 0.5 }));
    seq.add_keyframe(1500, ell_obj.move({ 0.2, 0, 0 }));
    seq.add_keyframe(2000, ell_obj.erase());

    seq.print_keyframes();

    const auto& c = seq.rasterize<5>(4, 12, 3000);  // 5x5x5 cube
    c.write("test.cbi");

    return 0;
}
