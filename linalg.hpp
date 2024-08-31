#pragma once

namespace linalg {

    struct vec3 {
        double x, y, z;
        // TODO would be much nicer with operator overloading (or with Eigen, I just don't want to risk subtle bugs by overlooking a discrepancy)
    };

    struct mat3x3 {
        double  v_0_0, v_0_1, v_0_2,
            v_1_0, v_1_1, v_1_2,
            v_2_0, v_2_1, v_2_2;
    };

    struct quat {
        double x, y, z, w;
    };

    static inline void scalar_multiply(const vec3& v, const double s, vec3& out) {
        out.x = v.x * s;
        out.y = v.y * s;
        out.z = v.z * s;
    }

    static inline void scalar_multiply(const quat& q, const double s, quat& out) {
        out.x = q.x * s;
        out.y = q.y * s;
        out.z = q.z * s;
        out.w = q.w * s;
    }

    static inline void scalar_multiply(vec3& v, const double s) {
        v.x *= s;
        v.y *= s;
        v.z *= s;
    }

    static inline void scalar_multiply(quat& q, const double s) {
        q.x *= s;
        q.y *= s;
        q.z *= s;
        q.w *= s;
    }

    static inline void scalar_multiply_accumulate(const vec3& v, double s, vec3& out) {
        out.x += v.x * s;
        out.y += v.y * s;
        out.z += v.z * s;
    }

    static inline void scalar_multiply_accumulate(const quat& v, double s, quat& out) {
        out.x += v.x * s;
        out.y += v.y * s;
        out.z += v.z * s;
        out.w += v.w * s;
    }

    static inline void cross_product(const vec3& v1, const vec3& v2, vec3& out) {
        out.x = v1.y * v2.z - v1.z * v2.y;
        out.y = v1.z * v2.x - v1.x * v2.z;
        out.z = v1.x * v2.y - v1.y * v2.x;
    }

    static inline void cross_product(const quat& v1, const vec3& v2, vec3& out) {
        out.x = v1.y * v2.z - v1.z * v2.y;
        out.y = v1.z * v2.x - v1.x * v2.z;
        out.z = v1.x * v2.y - v1.y * v2.x;
    }

    static inline void cross_product_accumulate(const vec3& v1, const vec3& v2, vec3& out) {
        out.x += v1.y * v2.z - v1.z * v2.y;
        out.y += v1.z * v2.x - v1.x * v2.z;
        out.z += v1.x * v2.y - v1.y * v2.x;
    }

    static inline void add(const vec3& v1, const vec3& v2, vec3& out) {
        out.x = v1.x + v2.x;
        out.y = v1.y + v2.y;
        out.z = v1.z + v2.z;
    }

    static inline void add_accumulate(const vec3& v1, const vec3& v2, vec3& out) {
        out.x += v1.x + v2.x;
        out.y += v1.y + v2.y;
        out.z += v1.z + v2.z;
    }

    static inline void add_accumulate(const vec3& v, vec3& out) {
        out.x += v.x;
        out.y += v.y;
        out.z += v.z;
    }

    static inline void add_accumulate(const quat& v, quat& out) {
        out.x += v.x;
        out.y += v.y;
        out.z += v.z;
        out.w += v.w;
    }

    static inline void sub(const vec3& v1, const vec3& v2, vec3& out) {
        out.x = v1.x - v2.x;
        out.y = v1.y - v2.y;
        out.z = v1.z - v2.z;
    }

    static inline void sub_accumulate(const vec3& v1, const vec3& v2, vec3& out) {
        out.x += v1.x - v2.x;
        out.y += v1.y - v2.y;
        out.z += v1.z - v2.z;
    }

    static inline void sub_accumulate(const vec3& v, vec3& out) {
        out.x -= v.x;
        out.y -= v.y;
        out.z -= v.z;
    }

    static inline void matrix_vector_product(const mat3x3& A, const vec3& v, vec3& out) {
        out.x = A.v_0_0 * v.x + A.v_0_1 * v.y + A.v_0_2 * v.z;
        out.y = A.v_1_0 * v.x + A.v_1_1 * v.y + A.v_1_2 * v.z;
        out.z = A.v_2_0 * v.x + A.v_2_1 * v.y + A.v_2_2 * v.z;
    }

    static inline void quaternion_derivative(const quat& q, const vec3& omega, quat& q_dot) {
        q_dot.x = q.w * omega.x + q.y * omega.z - q.z * omega.y;
        q_dot.y = q.w * omega.y + q.z * omega.x - q.x * omega.z;
        q_dot.z = q.w * omega.z + q.x * omega.y - q.y * omega.x;
        q_dot.w = -q.x * omega.x - q.y * omega.y - q.z * omega.z;
        scalar_multiply(q_dot, 0.5);
    }

    static inline void rotate_vector_by_quaternion(const quat& q, const vec3& v, vec3& v_out) {
        vec3 var;
        cross_product(q, v, var);
        scalar_multiply(var, 2);
        cross_product(q, var, v_out);
        scalar_multiply_accumulate(var, q.w, v_out);
        add_accumulate(v, v_out);
    }
}
