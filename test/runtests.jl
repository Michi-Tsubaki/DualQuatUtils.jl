using Test
using DualQuatUtils
using LinearAlgebra, StaticArrays

@testset "Quaternion Operations" begin
    # Create quaternions for testing
    q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
    q2 = Quaternion(2.0, 3.0, 4.0, 5.0)
    q_identity = Quaternion(1.0, 0.0, 0.0, 0.0)
    q_pure = Quaternion(0.0, 1.0, 2.0, 3.0)
    
    # Test basic getter functions
    @test get_scalar(q1) == 1.0
    @test get_vector(q1) == SVector(2.0, 3.0, 4.0)
    
    # Test is_real and is_pure
    @test is_real(q_identity) == true
    @test is_real(q1) == false
    @test is_pure(q_pure) == true
    @test is_pure(q1) == false
    
    # Test norm
    @test norm(q1) ≈ sqrt(30.0)
    @test norm(q_identity) ≈ 1.0
    
    # Test normalization
    q1_norm = normalize(q1)
    @test norm(q1_norm) ≈ 1.0
    
    # Test conjugate
    q1_conj = conjugate(q1)
    @test q1_conj.w == q1.w
    @test q1_conj.x == -q1.x
    @test q1_conj.y == -q1.y
    @test q1_conj.z == -q1.z
    
    # Test inverse
    q1_inv = inverse(q1)
    q_prod = q1 * q1_inv
    @test q_prod.w ≈ 1.0
    @test isapprox(q_prod.x, 0.0, atol=1e-10)
    @test isapprox(q_prod.y, 0.0, atol=1e-10)
    @test isapprox(q_prod.z, 0.0, atol=1e-10)
    
    # Test quaternion addition
    q_sum = q1 + q2
    @test q_sum.w == q1.w + q2.w
    @test q_sum.x == q1.x + q2.x
    @test q_sum.y == q1.y + q2.y
    @test q_sum.z == q1.z + q2.z
    
    # Test quaternion multiplication
    q_prod_manual = Quaternion(
        q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z,
        q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
        q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x,
        q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w
    )
    q_prod = q1 * q2
    @test q_prod.w ≈ q_prod_manual.w
    @test q_prod.x ≈ q_prod_manual.x
    @test q_prod.y ≈ q_prod_manual.y
    @test q_prod.z ≈ q_prod_manual.z
end

@testset "Rotation Operations" begin
    # Test creation of rotation matrices
    rot_x = rotation_x(pi/2)
    rot_y = rotation_y(pi/2)
    rot_z = rotation_z(pi/2)
    
    # Check if rotation matrices are valid
    @test is_valid_rotation(rot_x)
    @test is_valid_rotation(rot_y)
    @test is_valid_rotation(rot_z)
    
    # Check rotation results
    v = [1.0, 0.0, 0.0]
    @test rot_x * v ≈ [1.0, 0.0, 0.0]  # Rotation around X axis doesn't change vectors along X
    @test rot_y * v ≈ [0.0, 0.0, -1.0] # Rotation around Y axis rotates X vector to Z direction
    @test rot_z * v ≈ [0.0, 1.0, 0.0]  # Rotation around Z axis rotates X vector to Y direction
    
    # Test conversion between rotation and RPY
    roll, pitch, yaw = 0.1, 0.2, 0.3
    rot = rpy2rotation(roll, pitch, yaw)
    roll_out, pitch_out, yaw_out = rotation2rpy(rot)
    @test roll_out ≈ roll
    @test pitch_out ≈ pitch
    @test yaw_out ≈ yaw
    
    # Test Rodrigues rotation formula
    axis = [1.0, 0.0, 0.0]  # X axis
    angle = pi/2
    rod_rot = Rodrigues(axis, angle)
    @test rod_rot.matrix ≈ rot_x.matrix
end

@testset "Quaternion-Rotation Conversion" begin
    # Test conversion from quaternion to rotation matrix
    q = Quaternion(cos(pi/4), sin(pi/4), 0.0, 0.0)  # Rotation of π/2 around X axis
    rot = quat2rotation(q)
    @test is_valid_rotation(rot)
    
    # Test conversion from rotation matrix to quaternion
    rot_y = rotation_y(pi/3)
    q_from_rot = rotation2quat(rot_y)
    rot_from_q = quat2rotation(q_from_rot)
    @test rot_from_q.matrix ≈ rot_y.matrix
    
    # Test consistency of round-trip conversion
    q_orig = Quaternion(0.7071, 0.7071, 0.0, 0.0)
    rot = quat2rotation(q_orig)
    q_back = rotation2quat(rot)
    # Quaternion signs may differ while representing the same rotation, so check absolute values
    @test abs(q_back.w) ≈ abs(q_orig.w) atol=1e-4
    @test abs(q_back.x) ≈ abs(q_orig.x) atol=1e-4
    @test abs(q_back.y) ≈ abs(q_orig.y) atol=1e-4
    @test abs(q_back.z) ≈ abs(q_orig.z) atol=1e-4
end

@testset "Dual Quaternion Operations" begin
    # Create basic dual quaternions
    q_real = Quaternion(1.0, 0.0, 0.0, 0.0)
    q_dual = Quaternion(0.0, 1.0, 0.0, 0.0)
    dq = DualQuaternion(q_real, q_dual)
    
    # Create quaternion for rotation
    rot_q = Quaternion(cos(pi/4), sin(pi/4), 0.0, 0.0)  # Rotation of π/2 around X axis
    
    # Create translation vector
    trans = [1.0, 2.0, 3.0]
    
    # Create rotation dual quaternion
    rot_dq = rotation_dq(rot_q)
    @test rot_dq.real == rot_q
    @test rot_dq.dual == Quaternion(0.0, 0.0, 0.0, 0.0)
    
    # Create translation dual quaternion
    trans_dq = translation_dq(trans)
    @test trans_dq.real == Quaternion(1.0, 0.0, 0.0, 0.0)
    @test trans_dq.dual.x ≈ 0.5 * trans[1]
    @test trans_dq.dual.y ≈ 0.5 * trans[2]
    @test trans_dq.dual.z ≈ 0.5 * trans[3]
    
    # Create and extract transform dual quaternion
    transform = transform_dq(rot_q, trans)
    extracted_rot, extracted_trans = extract_transform(transform)
    @test extracted_rot ≈ rot_q
    @test extracted_trans ≈ trans
    
    # Convert between dual quaternion and homogeneous transformation matrix
    H = dq2H(transform)
    dq_back = H2dq(H)
    # Compare rotation parts
    @test abs(dq_back.real.w) ≈ abs(transform.real.w)
    @test abs(dq_back.real.x) ≈ abs(transform.real.x)
    @test abs(dq_back.real.y) ≈ abs(transform.real.y)
    @test abs(dq_back.real.z) ≈ abs(transform.real.z)
    # Compare translation parts (extracted from homogeneous transformation matrix)
    _, trans_back = extract_transform(dq_back)
    @test trans_back ≈ trans
end

println("All tests passed!")