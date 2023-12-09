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

#ifndef VERBOSE_LOGGING
#define VERBOSE_LOGGING 0
#endif

/**
 * @brief Represents a 3D vector with x, y, and z coordinates.
 */
struct vec3 {
    double x, y, z;  // x, y, z coordinates
};

/**
 * @brief Represents LED cube
 * 
 * @tparam N The size of the cube.
 */
template<size_t N>
class cubic {
private:
    std::array<std::array<std::array<uint8_t, N>, N>, N> voxels; // 3D array representing the voxels
    int32_t duration; // duration in milliseconds

public:
    cubic() = default;

    /**
     * Set the value of a voxel at the specified coordinates.
     * 
     * @param x The x-coordinate of the voxel.
     * @param y The y-coordinate of the voxel.
     * @param z The z-coordinate of the voxel.
     * @param value The value to set for the voxel.
     */
    void set(int32_t x, int32_t y, int32_t z, uint8_t value) {
        voxels[x][y][z] = value;
    }

    /**
     * Set the duration of the cubic object.
     * 
     * @param _duration The duration in milliseconds.
     */
    void set_duration(int32_t _duration) {
        duration = _duration;
    }

    /**
     * Get the value of a voxel at the specified coordinates.
     * 
     * @param x The x-coordinate of the voxel.
     * @param y The y-coordinate of the voxel.
     * @param z The z-coordinate of the voxel.
     * @return The value of the voxel.
     */
    int8_t get(int32_t x, int32_t y, int32_t z) const {
        return voxels[x][y][z];
    }

    /**
     * Get the duration of the cubic object.
     * 
     * @return The duration in milliseconds.
     */
    int32_t get_duration() const {
        return duration;
    }

    /**
     * Print the cubic object.
     */
    void print() const {
        std::cout << "cubic print" << std::endl;
        std::cout << "duration: " << duration << " ms" << std::endl;
        for (size_t z = 0; z < N; ++z) {
            for (size_t y = 0; y < N; ++y) {
                for (size_t x = 0; x < N; ++x) {
                    std::cout << static_cast<int32_t>(voxels[x][y][z]) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }
};

/**
 * @brief Represents a sequence of cubic objects.
 * 
 * @tparam N The size of the cube
 */
template<size_t N>
class cubic_sequence {
private:
    std::vector<cubic<N>> cubics; /**< The vector of cubic objects. */
    const std::array<uint8_t, 4> signature = { 0x43, 0x0d, 0x0a, 0x00 }; /**< The signature of the file format. */

public:

    /**
     * @brief Appends a cubic object to the sequence.
     * @param _cubic The cubic object to append.
     */
    void append(const cubic<N>& _cubic) {
        cubics.push_back(_cubic);
    }

    /**
     * @brief Writes the cubic sequence to a file.
     * @param filepath The path of the file to write to.
     * @return True if the write operation is successful, false otherwise.
     */
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

#if VERBOSE_LOGGING == 1
        std::cout << "Frame size: " << N << "x" << s << std::endl;
#endif

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

/**
 * @brief The base class for all shapes in the cubicgen library.
 * 
 * This class provides common functionality and interface for all shapes.
 */
class shape_interface {
protected:
    bool erased = false; /* Flag indicating if the shape is erased. */
    int64_t id; /* The unique identifier of the shape. */
    static std::random_device rd; /* Random device used for generating random numbers. */
    static std::mt19937 gen; /* Random number generator engine. */
    static std::uniform_int_distribution<int64_t> dis; /* Uniform distribution for generating random numbers. */

public:
    /**
     * @brief Default constructor.
     * 
     * Initializes the shape with a unique identifier.
     */
    shape_interface() {
        id = dis(gen);
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~shape_interface() {}

    /**
     * @brief Get the unique identifier of the shape.
     * 
     * @return The unique identifier of the shape.
     */
    virtual int64_t get_id() const {
        return id;
    }

    /**
     * @brief Check if the shape is erased.
     * 
     * @return True if the shape is erased, false otherwise.
     */
    virtual bool is_erased() const {
        return erased;
    }

    /**
     * @brief Create a clone of the shape.
     * 
     * @return A unique pointer to the cloned shape.
     */
    virtual std::unique_ptr<shape_interface> clone() const = 0;

    /**
     * @brief Scale the shape by a given scale factor.
     * 
     * @param scale_factor The scale factor to apply to the shape.
     * @return A unique pointer to the scaled shape.
     */
    virtual std::unique_ptr<shape_interface> scale(const vec3& scale_factor) = 0;

    /**
     * @brief Move the shape by a given move vector.
     * 
     * @param move_vector The move vector to apply to the shape.
     * @return A unique pointer to the moved shape.
     */
    virtual std::unique_ptr<shape_interface> move(const vec3& move_vector) = 0;

    /**
     * @brief Erase the shape.
     * 
     * @return A unique pointer to the erased shape.
     */
    virtual std::unique_ptr<shape_interface> erase() = 0;

    /**
     * @brief Check if the shape includes a given point.
     * 
     * @param point The point to check.
     * @return True if the shape includes the point, false otherwise.
     */
    virtual bool includes(const vec3& point) const = 0;

    /**
     * @brief Perform linear interpolation between this shape and another shape.
     * 
     * @param other The other shape to interpolate with.
     * @param t The interpolation parameter.
     * @return A unique pointer to the interpolated shape.
     */
    virtual std::unique_ptr<shape_interface> lerp(const shape_interface& other, double t) const = 0;

    /**
     * @brief Get a debug string representation of the shape.
     * 
     * @return The debug string representation of the shape.
     */
    virtual std::string debug() const = 0;

protected:
    /**
     * @brief Set the unique identifier of the shape.
     * 
     * @param _id The unique identifier to set.
     */
    void set_id(int64_t _id) {
        id = _id;
    }
};

std::random_device shape_interface::rd;
std::mt19937 shape_interface::gen{ shape_interface::rd() };
std::uniform_int_distribution<int64_t> shape_interface::dis{};

/**
 * @brief Represents a cuboid shape.
 * 
 * The cuboid class inherits from the shape_interface class and provides
 * functionality to create, manipulate, and query cuboids in 3D space.
 */
class cuboid : public shape_interface {
private:
    vec3 position, size;

public:
    /**
     * @brief Construct a new cuboid object.
     * 
     * @param position The position of the cuboid.
     * @param size The size of the cuboid.
     */
    cuboid(const vec3& position, const vec3& size) : position(position), size(size) {}

    /**
     * @brief Construct a new cuboid object
     * 
     * @param other 
     */
    cuboid(const cuboid& other) : position(other.position), size(other.size) {}

    /**
     * @brief Create a clone of the cuboid.
     * 
     * @return A unique pointer to the cloned cuboid.
     */
    std::unique_ptr<shape_interface> clone() const override {
        auto ptr = std::make_unique<cuboid>(*this);
        ptr->set_id(id);
        return ptr;
    }

    /**
     * @brief Scale the cuboid by a given scale factor.
     * 
     * @param scale_factor The scale factor to apply to the cuboid.
     * @return A unique pointer to the scaled cuboid.
     */
    std::unique_ptr<shape_interface> scale(const vec3& scale_factor) override {
        size.x *= scale_factor.x;
        size.y *= scale_factor.y;
        size.z *= scale_factor.z;

#if VERBOSE_LOGGING == 1
        std::cout << "scaled " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Move the cuboid by a given move vector.
     * 
     * @param move_vector The move vector to apply to the cuboid.
     * @return A unique pointer to the moved cuboid.
     */
    std::unique_ptr<shape_interface> move(const vec3& move_vector) override {
        position.x += move_vector.x;
        position.y += move_vector.y;
        position.z += move_vector.z;
    
#if VERBOSE_LOGGING == 1
        std::cout << "moved " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Erase the cuboid.
     * 
     * @return A unique pointer to the erased cuboid.
     */
    std::unique_ptr<shape_interface> erase() override {
        erased = true;

#if VERBOSE_LOGGING == 1
        std::cout << "erased " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Check if the cuboid includes a given point.
     * 
     * @param point The point to check.
     * @return True if the cuboid includes the point, false otherwise.
     */
    bool includes(const vec3& point) const override {
        return (point.x >= position.x - size.x / 2 && point.x <= position.x + size.x / 2 &&
                point.y >= position.y - size.y / 2 && point.y <= position.y + size.y / 2 &&
                point.z >= position.z - size.z / 2 && point.z <= position.z + size.z / 2);
    }

    /**
     * @brief Perform linear interpolation between this cuboid and another cuboid.
     * 
     * @param other The other cuboid to interpolate with.
     * @param t The interpolation parameter.
     * @return A unique pointer to the interpolated cuboid.
     */
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

    /**
     * @brief Get a debug string representation of the cuboid.
     * 
     * @return The debug string representation of the cuboid.
     */
    std::string debug() const override {
        return "cuboid, pos: (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " + std::to_string(position.z) + "), size: (" + std::to_string(size.x) + ", " + std::to_string(size.y) + ", " + std::to_string(size.z) + ")";
    }
};

/**
 * @brief Represents an ellipsoid shape.
 * 
 * The ellipsoid class inherits from the shape_interface class and provides
 * functionality to create, manipulate, and query ellipsoids in 3D space.
 */
class ellipsoid : public shape_interface {
private:
    vec3 position;
    vec3 radii;

public:
    /**
     * @brief Construct a new ellipsoid object.
     * 
     * @param position The position of the ellipsoid.
     * @param radii The radii of the ellipsoid.
     */
    ellipsoid(const vec3& position, const vec3& radii) : position(position), radii(radii) {}

    /**
     * @brief Construct a new ellipsoid object.
     * 
     * @param position The position of the ellipsoid.
     * @param radii The radii of the ellipsoid.
     */
    ellipsoid(const vec3& position, double radii) : position(position), radii({radii, radii, radii}) {}

    /**
     * @brief Construct a new ellipsoid object.
     * 
     * @param other 
     */
    ellipsoid(const ellipsoid& other) : position(other.position), radii(other.radii) {}

    /**
     * @brief Create a clone of the ellipsoid.
     * 
     * @return A unique pointer to the cloned ellipsoid.
     */
    std::unique_ptr<shape_interface> clone() const override {
        auto ptr = std::make_unique<ellipsoid>(*this);
        ptr->set_id(id);
        return ptr;
    }

    /**
     * @brief Scale the ellipsoid by a given scale factor.
     * 
     * @param scale_factor The scale factor to apply to the ellipsoid.
     * @return A unique pointer to the scaled ellipsoid.
     */
    std::unique_ptr<shape_interface> scale(const vec3& scale_factor) override {
        radii.x *= scale_factor.x;
        radii.y *= scale_factor.y;
        radii.z *= scale_factor.z;

#if VERBOSE_LOGGING == 1
        std::cout << "scaled " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Move the ellipsoid by a given move vector.
     * 
     * @param move_vector The move vector to apply to the ellipsoid.
     * @return A unique pointer to the moved ellipsoid.
     */
    std::unique_ptr<shape_interface> move(const vec3& move_vector) override {
        position.x += move_vector.x;
        position.y += move_vector.y;
        position.z += move_vector.z;

#if VERBOSE_LOGGING == 1
        std::cout << "moved " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Erase the ellipsoid.
     * 
     * @return A unique pointer to the erased ellipsoid.
     */
    std::unique_ptr<shape_interface> erase() override {
        erased = true;

#if VERBOSE_LOGGING == 1
        std::cout << "erased " << std::to_string(id) << ", " << debug() << std::endl;
#endif
        return clone();
    }

    /**
     * @brief Check if the ellipsoid includes a given point.
     * 
     * @param point The point to check.
     * @return True if the ellipsoid includes the point, false otherwise.
     */
    bool includes(const vec3& point) const override {
        double dx = (point.x - position.x) / radii.x;
        double dy = (point.y - position.y) / radii.y;
        double dz = (point.z - position.z) / radii.z;
        return (dx * dx + dy * dy + dz * dz <= 1.0);
    }

    /**
     * @brief Perform linear interpolation between this ellipsoid and another ellipsoid.
     * 
     * @param other The other ellipsoid to interpolate with.
     * @param t The interpolation parameter.
     * @return A unique pointer to the interpolated ellipsoid.
     */
    std::unique_ptr<shape_interface> lerp(const shape_interface& other, double t) const override {
        // check if the other shape has the same id
        if (id != other.get_id()) {
            throw std::runtime_error("Cannot interpolate between two different shapes.");
        }

        const ellipsoid& other_ellipsoid = dynamic_cast<const ellipsoid&>(other);
    
#if VERBOSE_LOGGING == 1
        std::cout << "> lerp from " << debug() << std::endl;
        std::cout << "> lerp to " << other_ellipsoid.debug() << std::endl;
        std::cout << "> t: " << t << std::endl;
#endif

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

    /**
     * @brief Get a debug string representation of the ellipsoid.
     * 
     * @return The debug string representation of the ellipsoid.
     */
    std::string debug() const override {
        return "ellipsoid, pos: (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " + std::to_string(position.z) + "), radii: (" + std::to_string(radii.x) + ", " + std::to_string(radii.y) + ", " + std::to_string(radii.z) + ")";
    }
};

/**
 * @brief Represents a shape sequence.
 * 
 * The sphere class inherits from the shape_interface class and provides
 * functionality to create, manipulate, and query spheres in 3D space.
 */
class shape_sequence {
private:
    std::vector<std::pair<int32_t, std::unique_ptr<shape_interface>>> keyframes;

public:
    /**
     * @brief Add a keyframe to the shape sequence.
     * 
     * @param time The time of the keyframe in milliseconds.
     * @param shape The shape to add to the keyframe.
     */
    void add_keyframe(int32_t time, std::unique_ptr<shape_interface>&& shape) {
        std::cout << "keyframe added; " << std::to_string(shape->get_id()) << ", " << shape->debug() << ", time: " << std::to_string(time) << std::endl;
        keyframes.push_back({time, std::move(shape)});
    }

    /**
     * @brief Add a keyframe to the shape sequence.
     * 
     * @param time The time of the keyframe in milliseconds.
     * @param shape The shape to add to the keyframe.
     */
    void add_keyframe(int32_t time, const shape_interface& shape) {
        std::cout << "keyframe added; " << std::to_string(shape.get_id()) << ", " << shape.debug() << ", time: " << std::to_string(time) << std::endl;
        keyframes.push_back({ time, shape.clone() });
    }

    /**
     * @brief Rasterize the shape sequence into a cubic sequence.
     * 
     * @tparam N The size of the cubic sequence.
     * @param depth The depth of the cubic sequence.
     * @param resolution The resolution of the cubic sequence.
     * @param until The time until which the sequence is rasterized.
     * @return The rasterized cubic sequence.
     */
    template<size_t N>
    cubic_sequence<N> rasterize(int32_t depth, int32_t resolution, int32_t until) {
        // This function rasterizes the animation into a cubic.
        // Each voxel is represented by a 4-bit integer.

        std::cout << "Rasterizing sequence..." << std::endl;

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

#if VERBOSE_LOGGING == 1
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
#endif

        // Fill the gap between keyframes
        for (int32_t time = 0; time < until; time += resolution) {
#if VERBOSE_LOGGING == 1
            std::cout << "Rasterizing frame " << time << " ms =================" << std::endl;
#endif

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

#if VERBOSE_LOGGING == 1
            // Display objects_in_this_frame
            for (const auto& shape : objects_in_this_frame) {
                std::cout << "id: " << shape->get_id() << std::endl;
                std::cout << "info: " << shape->debug() << std::endl;
            }
#endif

            // Rasterize this frame
            auto c = rasterize_frame<N>(objects_in_this_frame, depth);
            c.set_duration(resolution);

#if VERBOSE_LOGGING == 1
            c.print();
#endif

            result.append(c);
        }

        return result;
    }

    /**
     * @brief Rasterize the shape sequence into a cubic.
     * 
     * @tparam N The size of the cubic.
     * @param shapes The shapes to rasterize.
     * @param depth The depth of the cubic.
     * @return The rasterized cubic.
     */
    template<size_t N>
    cubic<N> rasterize_frame(const std::vector<std::unique_ptr<shape_interface>>& shapes, int32_t depth) {
        // This function rasterizes the animation into a cubic.
        // Each voxel is represented by a 4-bit integer.

        cubic<N> c{};

        for (size_t x = 0; x < N; ++x) {
            for (size_t y = 0; y < N; ++y) {
                for (size_t z = 0; z < N; ++z) {
                    const int32_t brightness = static_cast<int32_t>(calc_overlapping_percentage<N>(x, y, z, shapes, depth) * (std::pow(2, depth) - 1));
                    c.set(x, y, z, brightness);
                }
            }
        }

        return c;
    }

    /**
     * @brief Print the shape sequence.
     */
    void debug() {
        for (const auto& [time, shape] : keyframes) {
            std::cout << time << ": " << shape->get_id() << std::endl;
        }
    }

private:
    /**
     * @brief Calculate the percentage of the shape that overlaps with each voxel.
     * 
     * @tparam N The size of the cubic.
     * @param x The x-coordinate of the voxel.
     * @param y The y-coordinate of the voxel.
     * @param z The z-coordinate of the voxel.
     * @param shapes The shapes to calculate the overlapping percentage with.
     * @param depth The depth of the cubic.
     * @return The percentage of the shape that overlaps with each voxel.
     */
    template<size_t N>
    double calc_overlapping_percentage(size_t x, size_t y, size_t z, const std::vector<std::unique_ptr<shape_interface>>& shapes, int32_t depth) {
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
