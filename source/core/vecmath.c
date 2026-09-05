/*---------------------------------------------------------------------------------
 * vecmath.c -- Vec3/Quat/Mat3 operations. See vecmath.h for the full contract.
 *
 * Scalar only -- Old 3DS's ARM11 is VFPv2, no NEON. Divides are avoided where
 * a divisor would otherwise be reused (normalize helpers compute one
 * reciprocal, then multiply); sqrt is avoided wherever a squared-length
 * comparison is sufficient (near-zero-length checks below).
 *---------------------------------------------------------------------------------*/
#include "core/vecmath.h"
#include <math.h>

/* Below this squared length, treat a vector/quaternion as too short to
 * normalize safely and fall back to a defined default instead of dividing by
 * (near) zero. 1e-12f is (1e-6f)^2 -- i.e. this rejects anything shorter than
 * a micrometre-scale vector in this project's metre units, which is well
 * below any physically meaningful direction. */
#define VEC3_NORMALIZE_EPS_SQ (1e-12f)
#define QUAT_NORMALIZE_EPS_SQ (1e-12f)

/* ---- Vec3 ---------------------------------------------------------- */

Vec3 vec3_make(f32 x, f32 y, f32 z) {
    Vec3 v = { x, y, z };
    return v;
}

Vec3 vec3_zero(void) {
    Vec3 v = { 0.0f, 0.0f, 0.0f };
    return v;
}

Vec3 vec3_add(Vec3 a, Vec3 b) {
    return vec3_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vec3 vec3_sub(Vec3 a, Vec3 b) {
    return vec3_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vec3 vec3_scale(Vec3 a, f32 s) {
    return vec3_make(a.x * s, a.y * s, a.z * s);
}

Vec3 vec3_negate(Vec3 a) {
    return vec3_make(-a.x, -a.y, -a.z);
}

f32 vec3_dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return vec3_make(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

f32 vec3_length_sq(Vec3 a) {
    return vec3_dot(a, a);
}

f32 vec3_length(Vec3 a) {
    return sqrtf(vec3_length_sq(a));
}

Vec3 vec3_normalize(Vec3 a) {
    f32 len_sq = vec3_length_sq(a);
    if (len_sq < VEC3_NORMALIZE_EPS_SQ) {
        return vec3_zero();
    }
    /* One sqrt, one reciprocal, three multiplies -- never three divides. */
    f32 inv_len = 1.0f / sqrtf(len_sq);
    return vec3_scale(a, inv_len);
}

Vec3 vec3_lerp(Vec3 a, Vec3 b, f32 t) {
    return vec3_add(a, vec3_scale(vec3_sub(b, a), t));
}

Vec3 vec3_rotate_by_quat(Vec3 v, Quat q) {
    /* Optimized q*v*q^-1 for unit q: t = 2 * cross(q.xyz, v);
     * v' = v + q.w * t + cross(q.xyz, t). Avoids building the full matrix
     * or doing two quaternion multiplies. */
    Vec3 qv = vec3_make(q.x, q.y, q.z);
    Vec3 t = vec3_scale(vec3_cross(qv, v), 2.0f);
    Vec3 result = vec3_add(v, vec3_scale(t, q.w));
    result = vec3_add(result, vec3_cross(qv, t));
    return result;
}

/* ---- Quat ------------------------------------------------------------ */

Quat quat_identity(void) {
    Quat q = { 0.0f, 0.0f, 0.0f, 1.0f };
    return q;
}

Quat quat_from_axis_angle(Vec3 axis, f32 angle_rad) {
    Vec3 unit_axis = vec3_normalize(axis);
    if (vec3_length_sq(unit_axis) < VEC3_NORMALIZE_EPS_SQ) {
        /* Degenerate (zero) axis -- no rotation is well-defined. */
        return quat_identity();
    }
    f32 half = angle_rad * 0.5f;
    f32 s = sinf(half);
    f32 c = cosf(half);
    Quat q = { unit_axis.x * s, unit_axis.y * s, unit_axis.z * s, c };
    return q;
}

Quat quat_mul(Quat a, Quat b) {
    /* Hamilton product, (x, y, z, w) layout, w last -- matches
     * vec3_rotate_by_quat's convention above. */
    Quat r;
    r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return r;
}

Quat quat_normalize(Quat q) {
    f32 len_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (len_sq < QUAT_NORMALIZE_EPS_SQ) {
        /* Degenerate -- cannot recover a direction, fall back to identity
         * rather than propagate NaN/inf into every rotation downstream. */
        return quat_identity();
    }
    f32 inv_len = 1.0f / sqrtf(len_sq);
    Quat r = { q.x * inv_len, q.y * inv_len, q.z * inv_len, q.w * inv_len };
    return r;
}

Quat quat_integrate(Quat q, Vec3 omega, f32 dt) {
    /* q_dot = 0.5 * omega_quat * q, omega_quat = (omega.x, omega.y, omega.z, 0).
     * First-order forward-Euler step on the quaternion derivative; caller is
     * responsible for renormalizing (see header contract). */
    Quat omega_quat = { omega.x, omega.y, omega.z, 0.0f };
    Quat q_dot = quat_mul(omega_quat, q);
    f32 half_dt = 0.5f * dt;
    Quat r;
    r.x = q.x + q_dot.x * half_dt;
    r.y = q.y + q_dot.y * half_dt;
    r.z = q.z + q_dot.z * half_dt;
    r.w = q.w + q_dot.w * half_dt;
    return r;
}

Mat3 quat_to_mat3(Quat q) {
    f32 x = q.x, y = q.y, z = q.z, w = q.w;
    f32 x2 = x + x, y2 = y + y, z2 = z + z;
    f32 xx = x * x2, xy = x * y2, xz = x * z2;
    f32 yy = y * y2, yz = y * z2, zz = z * z2;
    f32 wx = w * x2, wy = w * y2, wz = w * z2;

    Mat3 m;
    m.m[0][0] = 1.0f - (yy + zz);
    m.m[0][1] = xy - wz;
    m.m[0][2] = xz + wy;

    m.m[1][0] = xy + wz;
    m.m[1][1] = 1.0f - (xx + zz);
    m.m[1][2] = yz - wx;

    m.m[2][0] = xz - wy;
    m.m[2][1] = yz + wx;
    m.m[2][2] = 1.0f - (xx + yy);
    return m;
}

Quat quat_slerp(Quat a, Quat b, f32 t) {
    f32 cos_half_theta = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;

    /* Take the shorter path around the 4D unit sphere. */
    if (cos_half_theta < 0.0f) {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        cos_half_theta = -cos_half_theta;
    }

    /* Nearly parallel: sin(half_theta) is near zero, so the slerp formula's
     * division would be near-zero-over-near-zero. Linear interpolation (then
     * renormalize) is numerically stable and visually identical at this
     * separation. Compared against the cosine directly -- no sqrt needed. */
    if (cos_half_theta > 0.9995f) {
        Quat r;
        r.x = a.x + (b.x - a.x) * t;
        r.y = a.y + (b.y - a.y) * t;
        r.z = a.z + (b.z - a.z) * t;
        r.w = a.w + (b.w - a.w) * t;
        return quat_normalize(r);
    }

    f32 half_theta = acosf(cos_half_theta);
    f32 sin_half_theta = sqrtf(1.0f - cos_half_theta * cos_half_theta);
    f32 inv_sin_half_theta = 1.0f / sin_half_theta;

    f32 ratio_a = sinf((1.0f - t) * half_theta) * inv_sin_half_theta;
    f32 ratio_b = sinf(t * half_theta) * inv_sin_half_theta;

    Quat r;
    r.x = a.x * ratio_a + b.x * ratio_b;
    r.y = a.y * ratio_a + b.y * ratio_b;
    r.z = a.z * ratio_a + b.z * ratio_b;
    r.w = a.w * ratio_a + b.w * ratio_b;
    return r;
}

/* ---- Mat3 -------------------------------------------------------------- */

Mat3 mat3_identity(void) {
    Mat3 m;
    int r, c;
    for (r = 0; r < 3; r++) {
        for (c = 0; c < 3; c++) {
            m.m[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
    return m;
}

Mat3 mat3_mul(Mat3 a, Mat3 b) {
    Mat3 r;
    int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            f32 sum = 0.0f;
            for (k = 0; k < 3; k++) {
                sum += a.m[i][k] * b.m[k][j];
            }
            r.m[i][j] = sum;
        }
    }
    return r;
}

Mat3 mat3_transpose(Mat3 a) {
    Mat3 r;
    int i, j;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

Vec3 mat3_mul_vec3(Mat3 m, Vec3 v) {
    return vec3_make(
        m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
        m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
        m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z
    );
}

Mat3 mat3_diagonal(f32 xx, f32 yy, f32 zz) {
    Mat3 m;
    int r, c;
    for (r = 0; r < 3; r++) {
        for (c = 0; c < 3; c++) {
            m.m[r][c] = 0.0f;
        }
    }
    m.m[0][0] = xx;
    m.m[1][1] = yy;
    m.m[2][2] = zz;
    return m;
}
