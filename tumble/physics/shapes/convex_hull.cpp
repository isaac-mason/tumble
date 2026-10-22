#include "tumble/physics/shapes/convex_hull.h"

#include "tumble/math/aabox.h"
#include "tumble/math/mat4f.h"
#include "tumble/math/scalar.h"
#include "tumble/math/vec4f.h"
#include "tumble/math/vec3f.h"
#include "tumble/physics/shapes/convex.h"
#include "tumble/util/alloc.h"
#include "tumble/util/assert.h"
#include "tumble/util/mem.h"

TUMBLE_NAMESPACE_BEGIN

namespace convex_hull {

using convex_hull_builder::ConvexHullBuilder;

/** everything create allocates, released on every way out */
struct CreateScratch {
    Vec3f *input_points;
    uint32_t num_input_points;
    ConvexHullBuilder builder;
    bool has_builder;
    collections::Vec<int32_t> vertex_map;
    collections::Vec<int32_t> neighboring_faces;
    collections::Vec<uint32_t> *neighbor_sets;
    uint32_t num_neighbor_sets;
};

static void release_data(ConvexHullData *data) {
    data->point_positions.release();
    data->shrunk_point_positions.release();
    data->point_num_faces.release();
    data->point_faces.release();
    data->faces.release();
    data->planes.release();
    data->vertex_indices.release();
    data->point_neighbors_start.release();
    data->point_neighbors.release();
    util::free(data);
}

static void release_scratch(CreateScratch &scratch) {
    util::free(scratch.input_points, scratch.num_input_points);
    if (scratch.has_builder) convex_hull_builder::release(scratch.builder);
    scratch.vertex_map.release();
    scratch.neighboring_faces.release();
    for (uint32_t i = 0; i < scratch.num_neighbor_sets; i++) scratch.neighbor_sets[i].release();
    util::free(scratch.neighbor_sets, scratch.num_neighbor_sets);
}

static ConvexHullData *fail(BuildResult *out_result, BuildResult result, CreateScratch &scratch, ConvexHullData *data) {
    if (out_result != nullptr) *out_result = result;
    release_scratch(scratch);
    if (data != nullptr) release_data(data);
    return nullptr;
}

static bool contains(const collections::Vec<uint32_t> &values, uint32_t value) {
    for (uint32_t i = 0; i < values.length; i++) {
        if (values.data[i] == value) return true;
    }
    return false;
}

ConvexHullData *build(const ConvexHullBuildSettings &settings, BuildResult *out_result) {
    const float hull_tolerance = settings.hull_tolerance;
    const float convex_radius = settings.convex_radius;

    CreateScratch scratch;
    util::zero(&scratch, sizeof(CreateScratch));

    // validate convex radius
    if (convex_radius < 0.0f) return fail(out_result, BUILD_INVALID_CONVEX_RADIUS, scratch, nullptr);

    // convert flat positions array to Float3[]
    const uint32_t num_input_points = settings.num_positions;
    if (num_input_points > 0) {
        scratch.input_points = util::alloc<Vec3f>(num_input_points);
        if (scratch.input_points == nullptr) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, nullptr);
        scratch.num_input_points = num_input_points;
        for (uint32_t i = 0; i < num_input_points; i++) scratch.input_points[i] = vec3f::load(settings.positions, i * 3);
    }
    const Vec3f *input_points = scratch.input_points;

    // build convex hull
    scratch.builder = convex_hull_builder::create(input_points, num_input_points);
    scratch.has_builder = true;
    ConvexHullBuilder &builder = scratch.builder;
    const convex_hull_builder::Result init_result = convex_hull_builder::initialize(builder, MAX_POINTS_IN_HULL, hull_tolerance);

    if (init_result != convex_hull_builder::SUCCESS && init_result != convex_hull_builder::MAX_VERTICES_REACHED) {
        return fail(out_result, (BuildResult)init_result, scratch, nullptr);
    }

    // check the consistency of the resulting hull if we fully built it
    if (init_result == convex_hull_builder::SUCCESS) {
        convex_hull_builder::MaxError max_error;
        convex_hull_builder::determine_max_error(max_error, builder);
        // coplanar distance could be bigger than the allowed tolerance if the points are far apart
        if (max_error.max_error > 4.0f * scalar::max(max_error.coplanar_distance, hull_tolerance)) {
            return fail(out_result, BUILD_HULL_ERROR_TOO_LARGE, scratch, nullptr);
        }
    }

    ConvexHullData *data = util::alloc<ConvexHullData>();
    if (data == nullptr) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, nullptr);
    util::zero(data, sizeof(ConvexHullData));

    // get center of mass and volume
    Vec3f center_of_mass;
    float volume;
    convex_hull_builder::get_center_of_mass_and_volume(builder, center_of_mass, volume);

    // calculate covariance matrix for inertia tensor
    // canonical covariance matrix for unit tetrahedron
    // see: http://number-none.com/blow/inertia/deriving_i.html
    // clang-format off
    const Mat4f covariance_canonical = mat4f::make_columns(
        vec4f::make(1.0f / 60.0f , 1.0f / 120.0f, 1.0f / 120.0f, 0.0f),
        vec4f::make(1.0f / 120.0f, 1.0f / 60.0f , 1.0f / 120.0f, 0.0f),
        vec4f::make(1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 60.0f , 0.0f),
        vec4f::make(0.0f,          0.0f,          0.0f,          1.0f));
    // clang-format on
    Mat4f covariance_matrix = mat4f::zero();

    const collections::Vec<int32_t> &faces = builder.faces;
    const collections::Vec<convex_hull_builder::Edge> &edges = builder.edge_pool;
    Mat4f covariance_temp, covariance_transposed, covariance_result;
    for (uint32_t fi = 0; fi < faces.length; fi++) {
        const int32_t first_edge = builder.face_pool[faces[fi]].first_edge;

        // fourth point of tetrahedron is at center of mass
        // subtract center of mass from other points to get tetrahedron with one vertex at origin
        int32_t edge = first_edge;
        const Vec3f tetrahedron_vertex1 = vec3f::subtract(input_points[edges[edge].start_idx], center_of_mass);
        Vec3f tetrahedron_vertex2, tetrahedron_vertex3;

        // get second point
        edge = edges[edge].next_edge;
        tetrahedron_vertex2 = vec3f::subtract(input_points[edges[edge].start_idx], center_of_mass);

        // loop over triangle fan
        for (edge = edges[edge].next_edge; edge != first_edge; edge = edges[edge].next_edge) {
            tetrahedron_vertex3 = vec3f::subtract(input_points[edges[edge].start_idx], center_of_mass);
            const Vec3f v1 = tetrahedron_vertex1;
            const Vec3f v2 = tetrahedron_vertex2;
            const Vec3f v3 = tetrahedron_vertex3;

            // affine transform that transforms unit tetrahedron to this tetrahedron
            const Mat4f a = mat4f::make_columns(vec4f::make(vec3f::x(v1), vec3f::y(v1), vec3f::z(v1), 0.0f), vec4f::make(vec3f::x(v2), vec3f::y(v2), vec3f::z(v2), 0.0f),
                                                vec4f::make(vec3f::x(v3), vec3f::y(v3), vec3f::z(v3), 0.0f), vec4f::make(0.0f, 0.0f, 0.0f, 1.0f));

            // calculate determinant (3x3 part)
            const float det_a = vec3f::x(v1) * (vec3f::y(v2) * vec3f::z(v3) - vec3f::z(v2) * vec3f::y(v3)) - vec3f::y(v1) * (vec3f::x(v2) * vec3f::z(v3) - vec3f::z(v2) * vec3f::x(v3)) + vec3f::z(v1) * (vec3f::x(v2) * vec3f::y(v3) - vec3f::y(v2) * vec3f::x(v3));

            // calculate covariance matrix for this tetrahedron: det_a * (a * covariance_canonical * a^T)
            covariance_temp = mat4f::multiply(a, covariance_canonical);
            covariance_transposed = mat4f::transpose(a);
            covariance_result = mat4f::multiply(covariance_temp, covariance_transposed);
            covariance_result = mat4f::multiply_scalar(covariance_result, det_a);

            // add to total covariance matrix
            covariance_matrix = mat4f::add(covariance_matrix, covariance_result);

            // prepare for next triangle
            tetrahedron_vertex2 = tetrahedron_vertex3;
        }
    }

    // calculate inertia matrix: I = identity * trace(C) - C
    const float trace = v4f::lane<0>(covariance_matrix.c[0]) + v4f::lane<1>(covariance_matrix.c[1]) + v4f::lane<2>(covariance_matrix.c[2]);
    mat4f::store(data->inertia, mat4f::subtract(mat4f::multiply_scalar(mat4f::identity(), trace), covariance_matrix));

    // convert polygons from the builder into the shape's flat vertex arrays
    uint32_t total_face_vertices = 0;
    for (uint32_t fi = 0; fi < faces.length; fi++) {
        const int32_t first_edge = builder.face_pool[faces[fi]].first_edge;
        int32_t edge = first_edge;
        do {
            total_face_vertices++;
            edge = edges[edge].next_edge;
        } while (edge != first_edge);
    }
    // a vertex appears on at least one face, so the face vertex count bounds the point count
    const uint32_t max_points = total_face_vertices < num_input_points ? total_face_vertices : num_input_points;
    if (!scratch.vertex_map.reserve(num_input_points) || !data->point_positions.reserve(max_points * 3) ||
        !data->point_num_faces.reserve(max_points) || !data->point_faces.reserve(max_points * 3) || !data->faces.reserve(faces.length) ||
        !data->planes.reserve(faces.length) || !data->vertex_indices.reserve(total_face_vertices)) {
        return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
    }
    for (uint32_t i = 0; i < num_input_points; i++) scratch.vertex_map.push(-1);

    AABox local_bounds = aabox::empty();
    uint32_t num_points = 0;

    for (uint32_t fi = 0; fi < faces.length; fi++) {
        const convex_hull_builder::Face &builder_face = builder.face_pool[faces[fi]];
        const uint32_t first_vertex = data->vertex_indices.length;
        uint32_t num_vertices = 0;

        // loop over vertices in face
        int32_t edge = builder_face.first_edge;
        do {
            const int32_t original_idx = edges[edge].start_idx;
            int32_t new_idx = scratch.vertex_map[(uint32_t)original_idx];

            if (new_idx == -1) {
                // store point in original shape space (not relative to center of mass)
                const Vec3f src = input_points[original_idx];

                // update local bounds
                local_bounds = aabox::expand_by_point(local_bounds, src);

                // append vertex to the packed arrays (num faces / faces filled in below)
                new_idx = (int32_t)num_points++;
                data->point_positions.push(vec3f::x(src));
                data->point_positions.push(vec3f::y(src));
                data->point_positions.push(vec3f::z(src));
                data->point_num_faces.push(0);
                data->point_faces.push(-1);
                data->point_faces.push(-1);
                data->point_faces.push(-1);
                scratch.vertex_map[(uint32_t)original_idx] = new_idx;
            }

            // append to vertex list
            data->vertex_indices.push((uint32_t)new_idx);
            num_vertices++;

            edge = edges[edge].next_edge;
        } while (edge != builder_face.first_edge);

        // add face
        data->faces.push({first_vertex, num_vertices});

        // add plane
        // plane equation: normal . (x - centroid) = 0  =>  normal . x = normal . centroid
        // standard form: normal . x + constant = 0  =>  constant = -normal . centroid
        const Vec3f normal = vec3f::normalize(builder_face.normal);
        Plane3f plane;
        vec3f::store(plane.normal, normal);
        plane.constant = -vec3f::dot(normal, builder_face.centroid);
        data->planes.push(plane);
    }
    data->num_points = num_points;

    // validate number of points
    if (num_points > (uint32_t)MAX_POINTS_IN_HULL) return fail(out_result, BUILD_TOO_MANY_POINTS, scratch, data);

    const ConvexHullFace *hull_faces = data->faces.data;
    const uint32_t num_faces = data->faces.length;
    const Plane3f *planes = data->planes.data;
    const uint32_t *vertex_indices = data->vertex_indices.data;
    float *point_positions = data->point_positions.data;
    uint32_t *point_num_faces = data->point_num_faces.data;
    int32_t *point_faces = data->point_faces.data;
    collections::Vec<int32_t> &neighboring_faces = scratch.neighboring_faces;

    // for each point, find neighboring faces for convex radius support
    const float cos_one_degree = scalar::cos(1.0f * scalar::DEGREES_TO_RADIANS);
    for (uint32_t p = 0; p < num_points; p++) {
        neighboring_faces.length = 0;

        for (uint32_t f = 0; f < num_faces; f++) {
            const ConvexHullFace &face = hull_faces[f];
            for (uint32_t v = 0; v < face.num_vertices; v++) {
                if (vertex_indices[face.first_vertex + v] == p) {
                    if (!neighboring_faces.push((int32_t)f)) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
                    break;
                }
            }
        }

        if (neighboring_faces.length < 2) return fail(out_result, BUILD_POINT_WITH_TOO_FEW_FACES, scratch, data);

        // find the 3 normals that form the largest tetrahedron
        // if volume < 5% of max possible (1.0), fall back to 2 normals
        // if 2 normals are too close (angle < 1 degree), fall back to 1 normal
        float biggest_volume = 0.05f;
        int32_t best3[3] = {-1, -1, -1};
        float smallest_dot = cos_one_degree;
        int32_t best2[2] = {-1, -1};

        for (uint32_t face1 = 0; face1 < neighboring_faces.length; face1++) {
            const Vec3f normal1 = vec3f::load(planes[neighboring_faces[face1]].normal);
            for (uint32_t face2 = face1 + 1; face2 < neighboring_faces.length; face2++) {
                const Vec3f normal2 = vec3f::load(planes[neighboring_faces[face2]].normal);
                const Vec3f face_normal_cross = vec3f::cross(normal1, normal2);
                const float dot = vec3f::dot(normal1, normal2);

                if (dot < smallest_dot) {
                    smallest_dot = dot;
                    best2[0] = neighboring_faces[face1];
                    best2[1] = neighboring_faces[face2];
                }

                for (uint32_t face3 = face2 + 1; face3 < neighboring_faces.length; face3++) {
                    const Vec3f normal3 = vec3f::load(planes[neighboring_faces[face3]].normal);
                    const float tetrahedron_volume = scalar::abs(vec3f::dot(face_normal_cross, normal3));
                    if (tetrahedron_volume > biggest_volume) {
                        biggest_volume = tetrahedron_volume;
                        best3[0] = neighboring_faces[face1];
                        best3[1] = neighboring_faces[face2];
                        best3[2] = neighboring_faces[face3];
                    }
                }
            }
        }

        // select best faces
        const uint32_t fb = p * 3;
        if (best3[0] != -1) {
            point_num_faces[p] = 3;
            point_faces[fb] = best3[0];
            point_faces[fb + 1] = best3[1];
            point_faces[fb + 2] = best3[2];
        } else if (best2[0] != -1) {
            point_num_faces[p] = 2;
            point_faces[fb] = best2[0];
            point_faces[fb + 1] = best2[1];
            point_faces[fb + 2] = -1;
        } else {
            point_num_faces[p] = 1;
            point_faces[fb] = neighboring_faces[0];
            point_faces[fb + 1] = -1;
            point_faces[fb + 2] = -1;
        }
    }

    // reduce convex radius if hull is too thin or has sharp edges
    float final_convex_radius = convex_radius;

    // pass 1: check hull thickness - ensure convex radius fits
    if (final_convex_radius > 0.0f) {
        float min_size = scalar::FLOAT_MAX;

        // for each plane, find the furthest point behind it (hull thickness in that direction)
        for (uint32_t f = 0; f < num_faces; f++) {
            const Plane3f &plane = planes[f];
            float max_dist = 0.0f;
            const float nx = plane.normal.x, ny = plane.normal.y, nz = plane.normal.z;
            for (uint32_t i = 0; i < num_points * 3; i += 3) {
                // point is always behind plane (hull is convex), so negate signed distance
                const float dist = -(nx * point_positions[i] + ny * point_positions[i + 1] + nz * point_positions[i + 2] + plane.constant);
                if (dist > max_dist) max_dist = dist;
            }
            min_size = scalar::min(min_size, max_dist);
        }

        // we need to fit 2x convex radius in min_size
        final_convex_radius = scalar::min(final_convex_radius, 0.5f * min_size);
    }

    // pass 2: check sharp edges - ensure vertices don't shift too far when planes are offset
    if (final_convex_radius > 0.0f) {
        const float max_error_convex_radius = settings.max_error_convex_radius;

        for (uint32_t p = 0; p < num_points; p++) {
            const uint32_t point_face_count = point_num_faces[p];
            // skip if only 1 face (no sharp edge, shifting back is simple)
            if (point_face_count == 1) continue;

            const uint32_t fb = p * 3;
            // get the 3 planes that define this vertex
            const Vec3f n1 = vec3f::load(planes[point_faces[fb]].normal);
            const Vec3f n2 = vec3f::load(planes[point_faces[fb + 1]].normal);
            Vec3f n3, offset_mask;

            if (point_face_count == 3) {
                // all 3 neighboring planes are offset by convex radius
                n3 = vec3f::load(planes[point_faces[fb + 2]].normal);
                offset_mask = vec3f::make(1.0f, 1.0f, 1.0f);
            } else {
                // only 2 planes, create perpendicular plane through vertex (its constant does not enter the shift)
                n3 = vec3f::normalize(vec3f::cross(n1, n2));
                // third plane not offset
                offset_mask = vec3f::make(1.0f, 1.0f, 0.0f);
            }

            // build matrix n with plane normals as rows (transposed from column form)
            const Mat4f n = mat4f::make_columns(vec4f::make(vec3f::x(n1), vec3f::x(n2), vec3f::x(n3), 0.0f), vec4f::make(vec3f::y(n1), vec3f::y(n2), vec3f::y(n3), 0.0f),
                                                vec4f::make(vec3f::z(n1), vec3f::z(n2), vec3f::z(n3), 0.0f), vec4f::make(0.0f, 0.0f, 0.0f, 1.0f));

            const float det_n = mat4f::determinant(n);
            if (det_n == 0.0f) {
                // n is not invertible: no way to move the vertex back, so no convex radius
                final_convex_radius = 0.0f;
                break;
            }

            // calculate shift direction: adjoint(n) x offset_mask / det(n)
            // the adjoint's w row is (0, 0, 0, det(n)), so transform_mat4's divide by w is the divide by det(n)
            const Mat4f normal_matrix_adjoint = mat4f::adjoint(n);
            const Vec3f vertex_shift_direction = vec3f::transform_mat4(offset_mask, normal_matrix_adjoint);
            const float offset = vec3f::length(vertex_shift_direction);

            // max radius: error = r * (offset - 1)  =>  r = error / (offset - 1)
            if (offset > 1.0f) {
                const float max_radius = max_error_convex_radius / (offset - 1.0f);
                final_convex_radius = scalar::min(final_convex_radius, max_radius);
            }
            // if offset <= 1, vertex moves inward, no constraint needed
        }
    }

    // compute the convex-radius-shrunk vertex set once; the exclude-mode support fill borrows it per pair.
    // with no convex radius the shrink is the identity and the fill reads point_positions instead.
    if (final_convex_radius != 0.0f) {
        if (!data->shrunk_point_positions.reserve(num_points * 3)) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
        support::compute_shrunk_hull_points(data->shrunk_point_positions, num_points, point_positions, point_num_faces, point_faces, planes,
                                            final_convex_radius);
    }

    // bake the vertex 1-ring adjacency (csr) for support hill climbing. one table serves every scale
    // path (affine transforms preserve hull vertex adjacency) and the shrunk point set (same count/order).
    const BakeSupportAdjacency bake = settings.bake_support_adjacency;
    const bool bake_adjacency = bake == BAKE_SUPPORT_ADJACENCY_AUTO ? num_points > SUPPORT_HILL_CLIMB_MIN_POINTS
                                                                    : bake == BAKE_SUPPORT_ADJACENCY_ALWAYS;
    if (bake_adjacency) {
        // walk every face's consecutive vertex pairs (each polygon edge, both directions); dedup per vertex
        scratch.neighbor_sets = util::alloc<collections::Vec<uint32_t>>(num_points);
        if (scratch.neighbor_sets == nullptr) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
        util::zero(scratch.neighbor_sets, num_points * (uint32_t)sizeof(collections::Vec<uint32_t>));
        scratch.num_neighbor_sets = num_points;
        collections::Vec<uint32_t> *neighbor_sets = scratch.neighbor_sets;

        for (uint32_t f = 0; f < num_faces; f++) {
            const uint32_t first = hull_faces[f].first_vertex;
            const uint32_t count = hull_faces[f].num_vertices;
            for (uint32_t v = 0; v < count; v++) {
                const uint32_t a = vertex_indices[first + v];
                const uint32_t b = vertex_indices[first + (v + 1) % count];
                if (!contains(neighbor_sets[a], b) && !neighbor_sets[a].push(b)) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
                if (!contains(neighbor_sets[b], a) && !neighbor_sets[b].push(a)) return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
            }
        }

        uint32_t total_neighbors = 0;
        for (uint32_t p = 0; p < num_points; p++) total_neighbors += neighbor_sets[p].length;
        if (!data->point_neighbors_start.reserve(num_points + 1) || !data->point_neighbors.reserve(total_neighbors)) {
            return fail(out_result, BUILD_OUT_OF_MEMORY, scratch, data);
        }
        data->point_neighbors_start.push(0);
        for (uint32_t p = 0; p < num_points; p++) {
            for (uint32_t i = 0; i < neighbor_sets[p].length; i++) data->point_neighbors.push(neighbor_sets[p].data[i]);
            data->point_neighbors_start.push(data->point_neighbors.length);
        }
    }

    data->convex_radius = final_convex_radius;
    aabox::store(data->aabb, local_bounds);
    vec3f::store(data->center_of_mass, center_of_mass);
    data->volume = volume;

    release_scratch(scratch);
    if (out_result != nullptr) *out_result = BUILD_SUCCESS;
    return data;
}

void release(ConvexHullData *data) { release_data(data); }

Shape *create(const ConvexHullShapeSettings &settings) {
    ConvexHullData *data = settings.data;
    TUMBLE_ASSERT(data != nullptr);
    ConvexHullShape *shape = shapes::allocate<ConvexHullShape>();
    if (shape == nullptr) return nullptr;
    shape->data = data;
    shape->density = settings.density;
    shape->owns_data = settings.owns_data;
    shape->material_id = settings.material_id;
    shape->aabb = data->aabb;
    shape->center_of_mass = data->center_of_mass;
    shape->volume = data->volume;
    return shape;
}

void destroy(Shape *shape) {
    const ConvexHullShape &hull = shapes::as<ConvexHullShape>(shape);
    if (hull.owns_data) release(hull.data);
    shapes::release<ConvexHullShape>(shape);
}

void update(Shape *shape) {
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;
    TUMBLE_ASSERT(data.convex_radius >= 0.0f);

    AABox bounds = aabox::empty();
    for (uint32_t p = 0; p < data.num_points; p++) bounds = aabox::expand_by_point(bounds, vec3f::load(data.point_positions.data, p * 3));
    aabox::store(shape->aabb, bounds);
}

void compute_mass_properties(mass_properties::MassProperties &out, const Shape *shape) {
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;

    // mass = volume * density
    out.mass = shape->volume * shapes::as<ConvexHullShape>(shape).density;

    // scale inertia tensor by density
    mat4f::store(out.inertia, mat4f::multiply_scalar(mat4f::load(data.inertia), shapes::as<ConvexHullShape>(shape).density));
}

void get_surface_normal(shapes::SurfaceNormalResult &io, const Shape *shape, sub_shape::SubShapeId sub_shape_id) {
    TUMBLE_ASSERT(sub_shape::is_empty(sub_shape_id));
    (void)sub_shape_id;
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;

    // find the plane with the smallest absolute signed distance to the surface point
    const Plane3f &first_plane = data.planes[0];
    io.normal = vec3f::load(first_plane.normal);
    float best_dist = scalar::abs(first_plane.constant + vec3f::dot(io.normal, io.position));

    for (uint32_t i = 1; i < data.planes.length; i++) {
        const Plane3f &plane = data.planes[i];
        const Vec3f normal = vec3f::load(plane.normal);
        const float dist = scalar::abs(plane.constant + vec3f::dot(normal, io.position));
        if (dist < best_dist) {
            best_dist = dist;
            io.normal = normal;
        }
    }
}

void get_supporting_face(shapes::SupportingFaceResult &io, Vec3f direction, const Shape *shape, sub_shape::SubShapeId) {
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;
    face::Face &face = io.face;
    const Vec3f scale = io.scale;

    const Plane3f *planes = data.planes.data;
    const uint32_t num_planes = data.planes.length;
    uint32_t best_face_idx = 0;

    if (vec3f::x(scale) == vec3f::y(scale) && vec3f::y(scale) == vec3f::z(scale)) {
        // uniform scale: |inv_scale . normal| is one constant across all planes (normals are unit at
        // build), so normalization cannot change the argmin; no per-plane sqrt. mirrored (negative)
        // scale flips every dot equally, folded into the sign.
        const float dx = vec3f::x(direction), dy = vec3f::y(direction), dz = vec3f::z(direction);
        const float sign = vec3f::x(scale) < 0.0f ? -1.0f : 1.0f;
        const Vec3f n0 = vec3f::load(planes[0].normal);
        float best_dot = sign * (vec3f::x(n0) * dx + vec3f::y(n0) * dy + vec3f::z(n0) * dz);
        for (uint32_t i = 1; i < num_planes; i++) {
            const Vec3f n = vec3f::load(planes[i].normal);
            const float dot = sign * (vec3f::x(n) * dx + vec3f::y(n) * dy + vec3f::z(n) * dz);
            if (dot < best_dot) {
                best_dot = dot;
                best_face_idx = i;
            }
        }
    } else {
        // non-uniform scale: normals transform by (M^-1)^T = 1/scale for a diagonal scale matrix
        const Vec3f inv_scale = vec3f::make(1.0f / vec3f::x(scale), 1.0f / vec3f::y(scale), 1.0f / vec3f::z(scale));

        // transform first plane normal and find initial best
        Vec3f plane_normal = vec3f::multiply(inv_scale, vec3f::load(planes[0].normal));
        const float plane0_normal_length = vec3f::length(plane_normal);
        float best_dot = vec3f::dot(plane_normal, direction) / plane0_normal_length;

        // find face with smallest (most negative) dot product
        for (uint32_t i = 1; i < num_planes; i++) {
            plane_normal = vec3f::multiply(inv_scale, vec3f::load(planes[i].normal));
            const float plane_normal_length = vec3f::length(plane_normal);
            const float dot = vec3f::dot(plane_normal, direction) / plane_normal_length;

            if (dot < best_dot) {
                best_dot = dot;
                best_face_idx = i;
            }
        }
    }

    // get vertices of best face
    const ConvexHullFace &best_face = data.faces[best_face_idx];
    const uint32_t first_vtx_idx = best_face.first_vertex;
    const int32_t num_vertices = (int32_t)best_face.num_vertices;

    // downsampled so contact clipping cannot overflow; every nth vertex, not the ones that matter most
    const int32_t max_vertices_to_return = (int32_t)(MAX_FACE_VERTICES / 2);
    const int32_t delta_vtx = (num_vertices + max_vertices_to_return - 1) / max_vertices_to_return;

    // check if scale inverts winding (negative determinant)
    const bool inside_out = face::is_scale_inside_out(scale);

    face.num_vertices = 0;
    const uint32_t *vertex_indices = data.vertex_indices.data;
    const float *point_positions = data.point_positions.data;

    if (inside_out) {
        // flip winding of supporting face
        for (int32_t i = num_vertices - 1; i >= 0; i -= delta_vtx) {
            face.vertices[face.num_vertices++] = vec3f::load(point_positions, vertex_indices[first_vtx_idx + (uint32_t)i] * 3);
        }
    } else {
        // normal winding of supporting face
        for (int32_t i = 0; i < num_vertices; i += delta_vtx) {
            face.vertices[face.num_vertices++] = vec3f::load(point_positions, vertex_indices[first_vtx_idx + (uint32_t)i] * 3);
        }
    }

    face::transform_with_mat4_scale(face, io.transform, scale);
}

float get_inner_radius(const Shape *shape) {
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;

    // calculate the inner radius by getting the minimum distance from the center of mass to the planes of the hull
    // (the planes are in input space, not relative to the center of mass)
    const Vec3f center_of_mass = vec3f::load(shape->center_of_mass);
    float inner_radius = scalar::FLOAT_MAX;
    for (uint32_t i = 0; i < data.planes.length; i++) {
        const Plane3f &plane = data.planes[i];
        inner_radius = scalar::min(inner_radius, -(vec3f::dot(vec3f::load(plane.normal), center_of_mass) + plane.constant));
    }
    // clamp against zero for numerical stability (flat convex hulls may have round-off issues)
    return scalar::max(0.0f, inner_radius);
}

void set_support(support::Support &out, const Shape *shape, support::SupportFunctionMode mode, Vec3f scale) {
    const ConvexHullData &data = *shapes::as<ConvexHullShape>(shape).data;
    out.get_support = support::hull_support;
    out.has_transform = false;
    out.add_radius = 0.0f;

    auto &hull = out.hull;
    hull.vertex_count = data.num_points;

    // borrow the shape's csr adjacency (empty means not baked: brute scan). valid for the shrunk set (same
    // count/order) and every scale path (affine transforms preserve hull vertex adjacency). reset the
    // warm-start hint so this pair-fill starts cold (first evaluation brute-seeds it).
    hull.neighbors_start = data.point_neighbors_start.length > 0 ? data.point_neighbors_start.data : nullptr;
    hull.neighbors = data.point_neighbors.data;
    hull.last_vertex = -1;

    const bool raw_vertices = mode == support::INCLUDE_CONVEX_RADIUS || data.convex_radius == 0.0f;

    // uniform positive scale (identity is just s = 1): borrow the shape-owned vertex arrays and scale the
    // winning support point in the evaluator; exact, and no per-pair bake. non-uniform / mirrored scale:
    // fall back to baking scaled vertices into scratch per pair (slow path).
    const bool uniform = vec3f::x(scale) == vec3f::y(scale) && vec3f::y(scale) == vec3f::z(scale) && vec3f::x(scale) > 0.0f;

    if (uniform) {
        const float s = vec3f::x(scale);
        hull.output_scale = s;
        if (raw_vertices) {
            out.convex_radius = 0.0f;
            hull.vertices = data.point_positions.data;
        } else {
            out.convex_radius = data.convex_radius * s;
            hull.vertices = data.shrunk_point_positions.data;
        }
        return;
    }

    // slow path: non-uniform / mirrored scale bakes per-pair vertices into scratch
    hull.output_scale = 1.0f;
    TUMBLE_ASSERT(data.num_points <= support::MAX_HULL_POINTS);

    if (raw_vertices) {
        out.convex_radius = 0.0f;
        // bake scaled raw vertices into scratch
        const float *positions = data.point_positions.data;
        float *scratch = hull.scratch;
        const uint32_t required_length = data.num_points * 3;
        for (uint32_t i = 0; i < required_length; i += 3) vec3f::store(scratch, vec3f::multiply(vec3f::load(positions, i), scale), i);
        hull.vertices = scratch;
    } else {
        // exclude convex radius (nonzero): scaled shrunk vertices
        support::compute_scaled_shrunk_hull_points(hull.scratch, data.num_points, data.point_positions.data, data.point_num_faces.data,
                                                   data.point_faces.data, data.planes.data, data.convex_radius, scale);
        out.convex_radius = support::scale_convex_radius(data.convex_radius, scale);
        hull.vertices = hull.scratch;
    }
}

shapes::ShapeDef def() {
    shapes::ShapeDef shape_def = {};
    shape_def.type = shapes::SHAPE_CONVEX_HULL;
    shape_def.set_support = set_support;
    shape_def.compute_mass_properties = compute_mass_properties;
    shape_def.get_surface_normal = get_surface_normal;
    shape_def.get_supporting_face = get_supporting_face;
    shape_def.get_material = convex::get_convex_material;
    shape_def.get_inner_radius = get_inner_radius;
    shape_def.cast_ray = convex::cast_ray_vs_convex;
    shape_def.collide_point = convex::collide_point_vs_convex;
    return shapes::define_shape(shape_def);
}

}  // namespace convex_hull

TUMBLE_NAMESPACE_END
