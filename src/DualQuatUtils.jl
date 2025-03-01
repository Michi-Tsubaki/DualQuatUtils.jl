module DualQuatUtils

using LinearAlgebra, StaticArrays

export Quaternion, get_scalar, get_vector, is_real, is_pure
export conjugate, qnorm, inverse, qnormalize
export quat2mat, rotation2rpy, rpy2rotation, skew_sym, Rodrigues
export quat2rotation, rotation2quat, is_valid_rotation
export rotation_x, rotation_y, rotation_z, visualise

export DualQuaternion, dual_conjugate, full_conjugate, one
export rotation_dq, translation_dq, transform_dq, extract_transform
export dq2H, H2dq

struct Quaternion <: FieldVector{4, Float64}
    w::Float64
    x::Float64
    y::Float64
    z::Float64
end

# Constructor
Quaternion(v::SVector{4}) = Quaternion(v[1], v[2], v[3], v[4])

import Base: show
function show(io::IO, q::Quaternion)
    print(io, "($(q.w) + $(q.x)i + $(q.y)j + $(q.z)k)")
end

# Scalar part
function get_scalar(q::Quaternion)
    return q.w
end

# Vector part
function get_vector(q::Quaternion)
    return SVector(q.x, q.y, q.z)
end

# Is real?
function is_real(q::Quaternion)
    if get_vector(q) == SVector(0.0, 0.0, 0.0)
        return true
    else
        return false
    end
end

# Is pure quaternion?
function is_pure(q::Quaternion)
    if get_scalar(q) == 0.0
        return true
    else
        return false
    end
end

function conjugate(q::Quaternion)
    return Quaternion(q.w, -q.x, -q.y, -q.z)
end

function qnorm(q::Quaternion)
    return (q.w^2 + q.x^2 + q.y^2 + q.z^2)^0.5
end

function inverse(q::Quaternion)
    return conjugate(q)/(qnorm(q)^2)
end

function qnormalize(q::Quaternion)
    return q/qnorm(q)
end

function quat2mat(q::Quaternion)
    mat = [q.w -q.x -q.y -q.z;
        q.x q.w -q.z q.y;
        q.y q.z q.w -q.x;
        q.z -q.y q.x q.w
        ]
    return SMatrix{4,4}(mat)
end

import Base: *
function *(q1::Quaternion, q2::Quaternion)
    return Quaternion(quat2mat(q1)*q2)
end

struct Rotation
    matrix::SMatrix{3, 3, Float64}
    
    Rotation() = new(@SMatrix [1.0 0.0 0.0; 0.0 1.0 0.0; 0.0 0.0 1.0])
    
    function Rotation(mat::AbstractMatrix)
        @assert size(mat) == (3, 3) "Rotation matrix must be 3x3"
        new(SMatrix{3,3}(mat))
    end
    
    function Rotation(m11, m12, m13, m21, m22, m23, m31, m32, m33)
        new(@SMatrix [m11 m12 m13; m21 m22 m23; m31 m32 m33])
    end
end

function *(r1::Rotation, r2::Rotation)
    Rotation(r1.matrix * r2.matrix)
end

# Apply rotation to a vector
function *(r::Rotation, v::AbstractVector)
    @assert length(v) == 3 "Vector must have length 3"
    r.matrix * v
end

function visualise(rotation::Rotation; 
                   scale=1.0, 
                   origin=[0,0,0], 
                   colors=[:red, :green, :blue],
                   show_original=true)
    # Original basis
    ex = [1.0, 0.0, 0.0] * scale
    ey = [0.0, 1.0, 0.0] * scale
    ez = [0.0, 0.0, 1.0] * scale
    
    # Rotated basis
    rex = rotation.matrix * ex
    rey = rotation.matrix * ey
    rez = rotation.matrix * ez
    
    # Origin
    o = collect(origin)
    
    # Visualization code that would be used with Plots package
    # This is a placeholder for actual visualization
    # In a real implementation, this would return a plot
    println("Visualization: Rotation matrix visualization")
    println("Original basis vectors (if show_original=$(show_original)):")
    println("ex = $ex")
    println("ey = $ey")
    println("ez = $ez")
    println("Rotated basis vectors:")
    println("rex = $rex")
    println("rey = $rey")
    println("rez = $rez")
    println("Origin: $o")
    
    # Return a symbolic representation since we can't actually plot here
    return Dict(
        "visualization_type" => "rotation",
        "original_basis" => (show_original ? [ex, ey, ez] : nothing),
        "rotated_basis" => [rex, rey, rez],
        "origin" => o
    )
end

# Check if rotation matrix is valid (orthogonal)
function is_valid_rotation(r::Rotation)
    mat = r.matrix
    identity_approx = mat * transpose(mat)
    return isapprox(identity_approx, I, atol=1e-10)
end

# roll - rotation around X axis
function rotation_x(angle_rad::Real)
    c = cos(angle_rad)
    s = sin(angle_rad)
    return Rotation(@SMatrix [
        1.0 0.0 0.0;
        0.0 c -s;
        0.0 s c
    ])
end

# pitch - rotation around Y axis
function rotation_y(angle_rad::Real)
    c = cos(angle_rad)
    s = sin(angle_rad)
    return Rotation(@SMatrix [
        c 0.0 s;
        0.0 1.0 0.0;
        -s 0.0 c
    ])
end

# yaw - rotation around Z axis
function rotation_z(angle_rad::Real)
    c = cos(angle_rad)
    s = sin(angle_rad)
    return Rotation(@SMatrix [
        c -s 0.0;
        s c 0.0;
        0.0 0.0 1.0
    ])
end

function rotation2rpy(rotation::Rotation)
    r = rotation.matrix # Extract matrix elements (needed for array indexing)
    # Check for gimbal lock
    # When cos(pitch) ≈ 0, i.e., pitch ≈ ±π/2
    if abs(r[3, 1]) > 1.0 - 1e-6
        # Handle gimbal lock
        # If r[3, 1] = -sin(pitch) is close to -1, then pitch ≈ π/2
        if r[3, 1] < 0
            roll = 0.0
            pitch = π/2
            yaw = atan(r[1, 2], r[2, 2])
        else
            # If r[3, 1] = -sin(pitch) is close to 1, then pitch ≈ -π/2
            roll = 0.0
            pitch = -π/2
            yaw = -atan(r[1, 2], r[2, 2])
        end
    else
        # Normal case
        roll = atan(r[3, 2] / r[3, 3]) # z
        pitch = -asin(r[3, 1]) # y
        yaw = atan(r[2, 1] / r[1, 1]) # x
    end
    
    return (roll, pitch, yaw)
end

function rpy2rotation(roll::Real, pitch::Real, yaw::Real)
    R = rotation_z(yaw) * rotation_y(pitch) * rotation_x(roll)
    return R
end

# eq. 65 - Create skew-symmetric matrix
function skew_sym(v::AbstractVector{<:Real})
    x, y, z = v
    
    mat = [0 -z y;
          z  0 -x;
         -y  x  0]
    
    return Rotation(mat)
end

# eq. 77 - Rodrigues rotation formula
function Rodrigues(v::AbstractVector{<:Real}, angle_rad::Real)
    # Normalize to unit vector
    u = v/norm(v)
    θ = angle_rad
    
    # skew_sym function returns a Rotation type
    # We need to access the internal matrix
    S = skew_sym(u).matrix
    
    # Rodrigues formula: R = I + sin(θ)S + (1-cos(θ))S²
    mat = I(3) + sin(θ)*S + (1-cos(θ))*(S*S)
    
    return Rotation(mat)
end

function quat2rotation(q::Quaternion)
    q = qnormalize(q)
    mat = [q.w^2+q.x^2-q.y^2-q.z^2 2(q.x*q.y-q.w*q.z) 2(q.x*q.z+q.w*q.y);
        2(q.x*q.y+q.w*q.z) q.w^2-q.x^2+q.y^2-q.z^2 2(q.y*q.z-q.w*q.x);
        2(q.x*q.z-q.w*q.y) 2(q.y*q.z+q.w*q.x) q.w^2-q.x^2-q.y^2+q.z^2]
    return Rotation(mat)
end

function visualise(q::Quaternion; scale=1.0, origin=[0,0,0], colors=[:red, :green, :blue], show_original=true)
    q_norm = qnormalize(q)
    rot = quat2rotation(q_norm)
    return visualise(rot; scale=scale, origin=origin, colors=colors, show_original=show_original)
end

function rotation2quat(r::Rotation)
    R = r.matrix
    # Calculate trace
    trace = R[1,1] + R[2,2] + R[3,3]
    
    if trace > 0
        # When trace is positive
        s = 0.5 / sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[3,2] - R[2,3]) * s
        y = (R[1,3] - R[3,1]) * s
        z = (R[2,1] - R[1,2]) * s
    elseif (R[1,1] > R[2,2]) && (R[1,1] > R[3,3])
        # When R[1,1] is maximum
        s = 2.0 * sqrt(1.0 + R[1,1] - R[2,2] - R[3,3])
        w = (R[3,2] - R[2,3]) / s
        x = 0.25 * s
        y = (R[1,2] + R[2,1]) / s
        z = (R[1,3] + R[3,1]) / s
    elseif R[2,2] > R[3,3]
        # When R[2,2] is maximum
        s = 2.0 * sqrt(1.0 + R[2,2] - R[1,1] - R[3,3])
        w = (R[1,3] - R[3,1]) / s
        x = (R[1,2] + R[2,1]) / s
        y = 0.25 * s
        z = (R[2,3] + R[3,2]) / s
    else
        # When R[3,3] is maximum
        s = 2.0 * sqrt(1.0 + R[3,3] - R[1,1] - R[2,2])
        w = (R[2,1] - R[1,2]) / s
        x = (R[1,3] + R[3,1]) / s
        y = (R[2,3] + R[3,2]) / s
        z = 0.25 * s
    end
    
    return Quaternion(w, x, y, z)
end

struct DualQuaternion
    real::Quaternion    # Real part
    dual::Quaternion    # Dual part
end

import Base: +
function +(dq1::DualQuaternion, dq2::DualQuaternion)
    real_part = dq1.real + dq2.real
    dual_part = dq1.dual + dq2.dual
    return DualQuaternion(real_part, dual_part)
end

import Base: -
function -(dq1::DualQuaternion, dq2::DualQuaternion)
    real_part = dq1.real - dq2.real
    dual_part = dq1.dual - dq2.dual
    return DualQuaternion(real_part, dual_part)
end

import Base: *
function *(s::Real, dq::DualQuaternion)
    return DualQuaternion(s * dq.real, s * dq.dual)
end

function *(dq::DualQuaternion, s::Real)
    return s * dq
end

function *(dq1::DualQuaternion, dq2::DualQuaternion)
    real_part = dq1.real * dq2.real
    dual_part = dq1.real * dq2.dual + dq1.dual * dq2.real
    return DualQuaternion(real_part, dual_part)
end

function conjugate(dq::DualQuaternion)
    return DualQuaternion(conjugate(dq.real), conjugate(dq.dual))
end

function dual_conjugate(dq::DualQuaternion)
    return DualQuaternion(dq.real, -dq.dual)
end

function full_conjugate(dq::DualQuaternion)
    conj_quat(q::Quaternion) = Quaternion(q.w, -q.x, -q.y, -q.z)
    return DualQuaternion(conj_quat(dq.real), -conj_quat(dq.dual))
end

function qnorm(dq::DualQuaternion)
    return sqrt(dq.real.w^2 + dq.real.x^2 + dq.real.y^2 + dq.real.z^2)
end

function qnormalize(dq::DualQuaternion)
    n = qnorm(dq)
    if n ≈ 0
        throw(DomainError(n, "Cannot normalize a dual quaternion with zero norm"))
    end
    
    n_inv = 1.0 / n
    real_part = n_inv * dq.real
    dual_part = n_inv * dq.dual
    
    # Calculate dot product of real and dual parts
    dot_prod = real_part.w * dual_part.w + real_part.x * dual_part.x + 
               real_part.y * dual_part.y + real_part.z * dual_part.z
    
    # Adjust dual_part so real and dual parts are orthogonal
    dual_part = dual_part - dot_prod * real_part
    
    return DualQuaternion(real_part, dual_part)
end

function one(::Type{DualQuaternion})
    return DualQuaternion(Quaternion(1.0, 0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 0.0))
end

function rotation_dq(q::Quaternion)
    q_norm=qnormalize(q)
    return DualQuaternion(q_norm, Quaternion(0.0, 0.0, 0.0, 0.0))
end

function translation_dq(v::Vector{Float64})
    if length(v) != 3
        throw(ArgumentError("Translation vector must have 3 elements"))
    end
    
    real_part = Quaternion(1.0, 0.0, 0.0, 0.0)
    
    dual_part = Quaternion(0.0, 0.5*v[1], 0.5*v[2], 0.5*v[3])
    
    return DualQuaternion(real_part, dual_part)
end

function transform_dq(q::Quaternion, v::Vector{Float64})
    if length(v) != 3
        throw(ArgumentError("Translation vector must have 3 elements"))
    end
    
    # Rotation
    rot_dq = rotation_dq(q)
    
    # Translation
    t = Quaternion(0.0, 0.5*v[1], 0.5*v[2], 0.5*v[3])
    trans_dual = t * rot_dq.real
    
    return DualQuaternion(rot_dq.real, trans_dual)
end

function extract_transform(dq::DualQuaternion)
    # Rotation
    rotation = dq.real
    
    # Translation
    t = 2.0 * (dq.dual * Quaternion(dq.real.w, -dq.real.x, -dq.real.y, -dq.real.z))
    translation = [t.x, t.y, t.z]
    
    return rotation, translation
end

function visualise(dq::DualQuaternion; scale=1.0, origin=[0,0,0], colors=[:red, :green, :blue], show_original=true, show_translation=true)
    # Extract rotation and translation from dual quaternion
    rotation, translation = extract_transform(dq)
    
    # Convert to rotation matrix
    rot_matrix = quat2rotation(rotation)
    
    # Original basis
    ex = [1.0, 0.0, 0.0] * scale
    ey = [0.0, 1.0, 0.0] * scale
    ez = [0.0, 0.0, 1.0] * scale
    
    # Rotated basis
    rex = rot_matrix.matrix * ex
    rey = rot_matrix.matrix * ey
    rez = rot_matrix.matrix * ez
    
    # Origin and transformed origin
    o = collect(origin)
    new_o = o + translation
    
    # Visualization code that would be used with Plots package
    # This is a placeholder for actual visualization
    println("Visualization: Dual Quaternion transformation")
    println("Original basis vectors (if show_original=$(show_original)):")
    println("ex = $ex")
    println("ey = $ey")
    println("ez = $ez")
    println("Transformed basis vectors:")
    println("rex = $rex")
    println("rey = $rey")
    println("rez = $rez")
    println("Original origin: $o")
    println("Transformed origin: $new_o")
    if show_translation
        println("Translation vector: $translation")
    end
    
    # Return a symbolic representation since we can't actually plot here
    return Dict(
        "visualization_type" => "dual_quaternion",
        "original_basis" => (show_original ? [ex, ey, ez] : nothing),
        "transformed_basis" => [rex, rey, rez],
        "original_origin" => o,
        "transformed_origin" => new_o,
        "translation" => (show_translation ? translation : nothing)
    )
end

function dq2H(dq::DualQuaternion)
    rotation, translation = extract_transform(dq)
    
    R = quat2rotation(rotation).matrix
    
    result = @SMatrix [
        R[1,1] R[1,2] R[1,3] translation[1];
        R[2,1] R[2,2] R[2,3] translation[2];
        R[3,1] R[3,2] R[3,3] translation[3];
        0.0    0.0    0.0    1.0
    ]
    
    return result
end

function H2dq(matrix::AbstractMatrix)  
    mat = @SMatrix [
        matrix[1,1] matrix[1,2] matrix[1,3];
        matrix[2,1] matrix[2,2] matrix[2,3];
        matrix[3,1] matrix[3,2] matrix[3,3]
    ]
    R = Rotation(mat)
    t = [matrix[1,4], matrix[2,4], matrix[3,4]]
    rot_q = rotation2quat(R)
    
    return transform_dq(rot_q, t)
end

end