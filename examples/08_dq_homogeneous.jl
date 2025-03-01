using DualQuatUtils
using LinearAlgebra, StaticArrays

# Demonstrate conversions between dual quaternions and homogeneous transformation matrices
println("Conversions between dual quaternions and homogeneous transformation matrices:")

# Create a rotation quaternion (Z-axis rotation of 45 degrees)
θ = π/4
axis = [0.0, 0.0, 1.0]
rot_q = Quaternion(cos(θ/2), sin(θ/2)*axis[1], sin(θ/2)*axis[2], sin(θ/2)*axis[3])

# Create a translation vector
trans = [2.0, 0.0, 1.0]

# Create a combined transformation dual quaternion
dq = transform_dq(rot_q, trans)
println("\nDual quaternion representation: DualQuaternion($(dq.real), $(dq.dual))")

# Convert to homogeneous transformation matrix
H = dq2H(dq)
println("\nHomogeneous transformation matrix:")
display(H)

# Convert back to dual quaternion
dq2 = H2dq(H)
println("\nConversion back to dual quaternion: DualQuaternion($(dq2.real), $(dq2.dual))")

# Compare original and reconstructed dual quaternions
println("\nComparing original and reconstructed dual quaternions:")
println("Original real part: $(dq.real)")
println("Reconstructed real part: $(dq2.real)")
println("Difference in real parts: $(norm(get_vector(dq.real) - get_vector(dq2.real)))")

println("\nOriginal dual part: $(dq.dual)")
println("Reconstructed dual part: $(dq2.dual)")
println("Difference in dual parts: $(norm(get_vector(dq.dual) - get_vector(dq2.dual)))")

# Visualize both transformations to confirm they're the same
println("\nVisualization of original dual quaternion transformation:")
result1 = visualise(dq)

println("\nVisualization of reconstructed dual quaternion transformation:")
result2 = visualise(dq2)

# Create a homogeneous transformation matrix from scratch
custom_H = @SMatrix [
    cos(θ) -sin(θ) 0.0 3.0;
    sin(θ) cos(θ) 0.0 1.0;
    0.0 0.0 1.0 2.0;
    0.0 0.0 0.0 1.0
]

println("\nCustom homogeneous transformation matrix:")
display(custom_H)

# Convert to dual quaternion
custom_dq = H2dq(custom_H)
println("\nCorresponding dual quaternion: DualQuaternion($(custom_dq.real), $(custom_dq.dual))")

# Visualize the transformation
println("\nVisualization of custom transformation:")
result3 = visualise(custom_dq)

# Convert back to matrix and check
custom_H2 = dq2H(custom_dq)
println("\nReconstruction error (Frobenius norm):")
println(norm(custom_H - custom_H2))