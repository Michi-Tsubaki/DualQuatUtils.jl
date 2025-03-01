using DualQuatUtils
using LinearAlgebra, StaticArrays

# Demonstrate dual quaternion transformations
println("Dual quaternion transformations:")

# Create a rotation quaternion (Z-axis rotation of 45 degrees)
θ = π/4
axis = [0.0, 0.0, 1.0]
rot_q = Quaternion(cos(θ/2), sin(θ/2)*axis[1], sin(θ/2)*axis[2], sin(θ/2)*axis[3])
println("\nRotation quaternion (45° around Z): $rot_q")

# Create a rotation dual quaternion
rot_dq = rotation_dq(rot_q)
println("\nRotation dual quaternion: DualQuaternion($(rot_dq.real), $(rot_dq.dual))")

# Create a translation vector
trans = [2.0, 0.0, 1.0]
println("\nTranslation vector: $trans")

# Create a translation dual quaternion
trans_dq = translation_dq(trans)
println("\nTranslation dual quaternion: DualQuaternion($(trans_dq.real), $(trans_dq.dual))")

# Create a combined transformation (rotation then translation)
dq = transform_dq(rot_q, trans)
println("\nCombined transformation dual quaternion: DualQuaternion($(dq.real), $(dq.dual))")

# Extract rotation and translation from dual quaternion
extracted_rotation, extracted_translation = extract_transform(dq)
println("\nExtracted rotation: $extracted_rotation")
println("Extracted translation: $extracted_translation")

# Visualize the transformation
println("\nVisualization of transformation:")
result = visualise(dq, scale=1.0)

# Example of applying a transformation to a point
function transform_point(dq::DualQuaternion, point::Vector{Float64})
    # Convert point to pure quaternion
    p_quat = Quaternion(0.0, point[1], point[2], point[3])
    
    # Create point dual quaternion
    p_dq = DualQuaternion(p_quat, Quaternion(0.0, 0.0, 0.0, 0.0))
    
    # Apply transformation: dq * p_dq * conjugate(dq)
    result_dq = dq * p_dq * conjugate(dq)
    
    # Extract result point
    return [result_dq.real.x, result_dq.real.y, result_dq.real.z]
end

# Test point transformation
test_point = [1.0, 1.0, 0.0]
transformed_point = transform_point(dq, test_point)

println("\nOriginal point: $test_point")
println("Transformed point: $transformed_point")