#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <random>
#include <bitset>

struct vec3 {
    double x, y, z;
};

// cubic is arbitary size of LED cube
// depth is 4-bit constant
template<size_t N>
class cubic {
private:
    std::array<std::array<std::array<uint8_t, N>, N>, N> voxels;
    int32_t duration;

public:
    cubic() = default;

    void set(int32_t x, int32_t y, int32_t z, uint8_t value) {
        voxels[x][y][z] = value;
    }

    void set_duration(int32_t _duration) {
        duration = _duration;
    }

    int8_t get(int32_t x, int32_t y, int32_t z) const {
        return voxels[x][y][z];
    }

    int32_t get_duration() const {
        return duration;
    }

    void print() const {
        std::cout << "cubic print" << std::endl;
        std::cout << "duration: " << duration << " ms" << std::endl;
        for (int32_t z = 0; z < N; ++z) {
            for (int32_t y = 0; y < N; ++y) {
                for (int32_t x = 0; x < N; ++x) {
                    std::cout << static_cast<int32_t>(voxels[x][y][z]) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }
};

template<size_t N>
class cubic_sequence {
private:
    std::vector<cubic<N>> cubics;
    const std::array<uint8_t, 4> signature = { 0x43, 0x0d, 0x0a, 0x00 };

public:
    void append(const cubic<N>& _cubic) {
        cubics.push_back(_cubic);
    }

    // Write to file
    // File format:
    // 4 bytes: signature (0x43, 0x0d, 0x0a, 0x00)
    // 4 bytes: frame count
    // ============= Frames ================
    // N * ceil(N * N / 2) bytes: frame
    // 4 bytes: duration
    // ===================================== x frame count
    bool write(const std::string& filepath) const {
        constexpr size_t s = std::ceil(1. * N * N / 2);
        std::ofstream writer(filepath, std::ios::binary);

        std::cout << "Writing to file: " << filepath << std::endl;

        if (!writer) {
            return false;
        }

        writer.write(reinterpret_cast<const char*>(signature.data()), signature.size());

        std::cout << "Frame count: " << cubics.size() << std::endl;

        uint32_t frame_count = static_cast<uint32_t>(cubics.size());
        writer.write(reinterpret_cast<const char*>(&frame_count), sizeof(uint32_t));

        std::cout << "Frame size: " << N << "x" << s << std::endl;

        for (const auto& cube : cubics) {
            uint8_t frame[N][s] = {};

            for (size_t z = 0; z < N; ++z) {
                for (size_t y = 0; y < N; ++y) {
                    for (size_t x = 0; x < N; ++x) {
                        const uint8_t value = cube.get(x, y, z) & 0x0f;
                        const size_t odd_shift = 4 * ((N * y + x) % 2);
                        frame[z][(N * y + x) / 2] |= value << odd_shift;
                    }
                }
            }

            writer.write(reinterpret_cast<const char*>(frame), N * s * sizeof(frame[0][0]));

            const int32_t duration = cube.get_duration();
            writer.write(reinterpret_cast<const char*>(&duration), sizeof(int32_t));
        }

        writer.flush();

        return true;
    }
};

class shape_interface {
protected:
    bool erased = false;
    int64_t id;
    static std::random_device rd;
    static std::mt19937 gen;
    static std::uniform_int_distribution<int64_t> dis;

public:
    shape_interface() {
        id = dis(gen);
    }
    virtual ~shape_interface() {}

    virtual int64_t get_id() const {
        return id;
    }

    virtual bool is_erased() const {
        return erased;
    }

    virtual std::unique_ptr<shape_interface> clone() const = 0;
    virtual std::unique_ptr<shape_interface> scale(const vec3& scale_factor) = 0;
    virtual std::unique_ptr<shape_interface> move(const vec3& move_vector) = 0;
    virtual std::unique_ptr<shape_interface> erase() = 0;
    virtual bool includes(const vec3& point) const = 0;
    virtual std::unique_ptr<shape_interface> lerp(const shape_interface& other, double t) const = 0;
    virtual std::string debug() const = 0;

protected:
    void set_id(int64_t _id) {
        id = _id;
    }
};

std::random_device shape_interface::rd;
std::mt19937 shape_interface::gen{ shape_interface::rd() };
std::uniform_int_distribution<int64_t> shape_interface::dis{};

class cuboid : public shape_interface {
private:
    vec3 position, size;

public:
    cuboid(const vec3& position, const vec3& size) : position(position), size(size) {}
    cuboid(const cuboid& other) : position(other.position), size(other.size) {}

    std::unique_ptr<shape_interface> clone() const override {
        auto ptr = std::make_unique<cuboid>(*this);
        ptr->set_id(id);
        return ptr;
    }

    std::unique_ptr<shape_interface> scale(const vec3& scale_factor) override {
        size.x *= scale_factor.x;
        size.y *= scale_factor.y;
        size.z *= scale_factor.z;
        std::cout << "scaled " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    std::unique_ptr<shape_interface> move(const vec3& move_vector) override {
        position.x += move_vector.x;
        position.y += move_vector.y;
        position.z += move_vector.z;
        std::cout << "moved " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    std::unique_ptr<shape_interface> erase() override {
        erased = true;
        std::cout << "erased " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    bool includes(const vec3& point) const override {
        return (point.x >= position.x && point.x <= position.x + size.x &&
                point.y >= position.y && point.y <= position.y + size.y &&
                point.z >= position.z && point.z <= position.z + size.z);
    }

    std::unique_ptr<shape_interface> lerp(const shape_interface& other, double t) const override {
        // check if the other shape has the same id
        if (id != other.get_id()) {
            throw std::runtime_error("Cannot interpolate between two different shapes.");
        }

        const cuboid& other_cuboid = dynamic_cast<const cuboid&>(other);

        auto result = std::make_unique<cuboid>(*this);
        result->set_id(id);
        result->position.x = position.x * (1 - t) + other_cuboid.position.x * t;
        result->position.y = position.y * (1 - t) + other_cuboid.position.y * t;
        result->position.z = position.z * (1 - t) + other_cuboid.position.z * t;
        result->size.x = size.x * (1 - t) + other_cuboid.size.x * t;
        result->size.y = size.y * (1 - t) + other_cuboid.size.y * t;
        result->size.z = size.z * (1 - t) + other_cuboid.size.z * t;
        return result;
    }

    std::string debug() const override {
        return "cuboid, pos: (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " + std::to_string(position.z) + "), size: (" + std::to_string(size.x) + ", " + std::to_string(size.y) + ", " + std::to_string(size.z) + ")";
    }
};

class ellipsoid : public shape_interface {
private:
    vec3 position;
    vec3 radii;

public:
    ellipsoid(const vec3& position, const vec3& radii) : position(position), radii(radii) {}
    ellipsoid(const vec3& position, double radii) : position(position), radii({radii, radii, radii}) {}
    ellipsoid(const ellipsoid& other) : position(other.position), radii(other.radii) {}

    std::unique_ptr<shape_interface> clone() const override {
        auto ptr = std::make_unique<ellipsoid>(*this);
        ptr->set_id(id);
        return ptr;
    }

    std::unique_ptr<shape_interface> scale(const vec3& scale_factor) override {
        radii.x *= scale_factor.x;
        radii.y *= scale_factor.y;
        radii.z *= scale_factor.z;
        std::cout << "scaled " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    std::unique_ptr<shape_interface> move(const vec3& move_vector) override {
        position.x += move_vector.x;
        position.y += move_vector.y;
        position.z += move_vector.z;
        std::cout << "moved " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    std::unique_ptr<shape_interface> erase() override {
        erased = true;
        std::cout << "erased " << std::to_string(id) << ", " << debug() << std::endl;
        return clone();
    }

    bool includes(const vec3& point) const override {
        double dx = (point.x - position.x) / radii.x;
        double dy = (point.y - position.y) / radii.y;
        double dz = (point.z - position.z) / radii.z;
        return (dx * dx + dy * dy + dz * dz <= 1.0);
    }

    std::unique_ptr<shape_interface> lerp(const shape_interface& other, double t) const override {
        // check if the other shape has the same id
        if (id != other.get_id()) {
            throw std::runtime_error("Cannot interpolate between two different shapes.");
        }

        const ellipsoid& other_ellipsoid = dynamic_cast<const ellipsoid&>(other);
        
        std::cout << "> lerp from " << debug() << std::endl;
        std::cout << "> lerp to " << other_ellipsoid.debug() << std::endl;
        std::cout << "> t: " << t << std::endl;

        auto result = std::make_unique<ellipsoid>(*this);
        result->set_id(id);
        result->position.x = position.x * (1 - t) + other_ellipsoid.position.x * t;
        result->position.y = position.y * (1 - t) + other_ellipsoid.position.y * t;
        result->position.z = position.z * (1 - t) + other_ellipsoid.position.z * t;
        result->radii.x = radii.x * (1 - t) + other_ellipsoid.radii.x * t;
        result->radii.y = radii.y * (1 - t) + other_ellipsoid.radii.y * t;
        result->radii.z = radii.z * (1 - t) + other_ellipsoid.radii.z * t;
        return result;
    }

    std::string debug() const override {
        return "ellipsoid, pos: (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " + std::to_string(position.z) + "), radii: (" + std::to_string(radii.x) + ", " + std::to_string(radii.y) + ", " + std::to_string(radii.z) + ")";
    }
};

class shape_sequence {
private:
    std::vector<std::pair<int32_t, std::unique_ptr<shape_interface>>> keyframes;

public:
    ~shape_sequence() = default;

    void add_keyframe(int32_t time, std::unique_ptr<shape_interface>&& shape) {
        std::cout << "keyframe added " << std::to_string(shape->get_id()) << ", " << shape->debug() << ", time: " << std::to_string(time) << std::endl;
        keyframes.push_back({time, std::move(shape)});
    }

    void add_keyframe(int32_t time, const shape_interface& shape) {
        std::cout << "keyframe added " << std::to_string(shape.get_id()) << ", " << shape.debug() << ", time: " << std::to_string(time) << std::endl;
        keyframes.push_back({ time, shape.clone() });
    }

    template<size_t N>
    cubic_sequence<N> rasterize(int32_t depth, int32_t resolution, int32_t until) {
        // This function rasterizes the animation into a cubic.
        // Each voxel is represented by a 4-bit integer.

        cubic_sequence<N> result{};

        std::sort(keyframes.begin(), keyframes.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        struct object_info {
            int64_t id;
            int32_t start_time;
            int32_t end_time;
            std::vector<std::pair<int32_t, std::unique_ptr<shape_interface>>> keyframes{};
        };

        // Group keyframes by object
        std::map<int64_t, object_info> objects;
        for (const auto& [time, shape] : keyframes) {
            const int64_t id = shape->get_id();
            if (objects.find(id) == objects.end()) {
                objects.emplace(id, object_info{ id, time, std::numeric_limits<int32_t>::max(), {} });
            }

            objects[id].keyframes.push_back({ time, shape->clone() });
            if (shape->is_erased()) {
                objects[id].end_time = time;
            }
        }

        // Be sure that all keyframes has 2 keyframes at least
        for (auto& [id, object] : objects) {
            if (object.keyframes.size() < 2) {
                object.keyframes.push_back({ object.end_time, object.keyframes.back().second->erase() });
            }
        }

        // Display objects
        std::cout << "Objects:" << std::endl;
        for (const auto& [id, object] : objects) {
            std::cout << "id: " << id << std::endl;
            std::cout << "start_time: " << object.start_time << std::endl;
            std::cout << "end_time: " << object.end_time << std::endl;
            std::cout << "keyframes: " << std::endl;
            for (const auto& [time, shape] : object.keyframes) {
                std::cout << "time: " << time << std::endl;
                std::cout << "shape: " << shape->debug() << std::endl;
            }
        }

        // Fill the gap between keyframes
        for (int32_t time = 0; time < until; time += resolution) {
            std::cout << "Rasterizing frame " << time << " ms =================" << std::endl;

            std::vector<std::unique_ptr<shape_interface>> objects_in_this_frame{};

            for (auto& [id, object] : objects) {
                if (object.start_time <= time && time <= object.end_time) {
                    int32_t prev_time = object.start_time;
                    auto prev_shape = object.keyframes[0].second->clone();

                    // Find the two keyframes that is closest to the current time
                    for (auto& [keyframe_time, shape] : object.keyframes) {
                        if (time == keyframe_time) { // If the keyframe time is the same as the current time
                            // Push the current keyframe
                            objects_in_this_frame.push_back(shape->clone());
                            break;
                        }
                        else if (keyframe_time < time) // If the keyframe time is smaller than the current time
                        {
                            // Store the previous keyframe
                            prev_time = keyframe_time;
                            prev_shape = shape->clone();
                        }
                        else if (time < keyframe_time) { // If the keyframe time is larger than the current time
                            // Interpolate between the previous keyframe and the next keyframe
                            const double t = static_cast<double>(time - prev_time) / (keyframe_time - prev_time);
                            objects_in_this_frame.push_back(prev_shape->lerp(*shape, t));
                            break;
                        }
                    }
                }
            }

            // Display objects_in_this_frame
            for (const auto& shape : objects_in_this_frame) {
                std::cout << "id: " << shape->get_id() << std::endl;
                std::cout << "info: " << shape->debug() << std::endl;
            }

            // Rasterize this frame
            auto c = rasterize_frame<N>(objects_in_this_frame, depth);
            c.set_duration(resolution);

            result.append(c);
        }

        return result;
    }

    template<size_t N>
    cubic<N> rasterize_frame(const std::vector<std::unique_ptr<shape_interface>>& shapes, int32_t depth) {
        // This function rasterizes the animation into a cubic.
        // Each voxel is represented by a 4-bit integer.

        cubic<N> c{};

        for (int32_t x = 0; x < N; ++x) {
            for (int32_t y = 0; y < N; ++y) {
                for (int32_t z = 0; z < N; ++z) {
                    const int32_t brightness = static_cast<int32_t>(calc_overlapping_percentage<N>(x, y, z, shapes, depth) * (std::pow(2, depth) - 1));
                    c.set(x, y, z, brightness);
                }
            }
        }

        return c;
    }

    void print_keyframes() {
        for (const auto& [time, shape] : keyframes) {
            std::cout << time << ": " << shape->get_id() << std::endl;
        }
    }

private:
    template<size_t N>
    double calc_overlapping_percentage(int32_t x, int32_t y, int32_t z, const std::vector<std::unique_ptr<shape_interface>>& shapes, int32_t depth) {
        // This function calculates the percentage of the shape that overlaps with each voxel.
        // The result is a 4-bit integer.

        const int32_t sampling_point_count = std::ceil(std::log(std::pow(2, depth) * 32) / std::log(3));
        const double sampling_size = 1. / (N * sampling_point_count);

        int32_t count = 0;

        for (int32_t xx = 0; xx < sampling_point_count; ++xx) {
            for (int32_t yy = 0; yy < sampling_point_count; ++yy) {
                for (int32_t zz = 0; zz < sampling_point_count; ++zz) {
                    const vec3 point = {
                        1. * x / N - 0.5 + (xx + 0.5) * sampling_size,
                        1. * y / N - 0.5 + (yy + 0.5) * sampling_size,
                        1. * z / N - 0.5 + (zz + 0.5) * sampling_size
                    };

                    bool included = false;
                    for (const auto& s : shapes)
                    {
                        if (s->includes(point))
                        {
                            included = true;
                            break;
                        }
                    }

                    if (included) {
                        count++;
                    }
                }
            }
        }

        return count / std::pow(sampling_point_count, 3);
    }
};
