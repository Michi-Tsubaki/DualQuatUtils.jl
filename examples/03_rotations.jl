using DualQuatUtils

using LinearAlgebra, StaticArrays

# Create basic rotations around each axis (45 degrees)
println("Basic rotation examples:")

# Roll (rotation around X axis)
rx = rotation_x(deg2rad(45))
println("\nRotation around X axis (roll) of 45 degrees:")
display(rx.matrix)
println("\nVisualization of roll rotation:")
result = visualise(rx)
println("Roll rotation is valid: ", is_valid_rotation(rx))

# Pitch (rotation around Y axis)
ry = rotation_y(deg2rad(45))
println("\nRotation around Y axis (pitch) of 45 degrees:")
display(ry.matrix)
println("\nVisualization of pitch rotation:")
result = visualise(ry)
println("Pitch rotation is valid: ", is_valid_rotation(ry))

# Yaw (rotation around Z axis)
rz = rotation_z(deg2rad(45))
println("\nRotation around Z axis (yaw) of 45 degrees:")
display(rz.matrix)
println("\nVisualization of yaw rotation:")
result = visualise(rz)
println("Yaw rotation is valid: ", is_valid_rotation(rz))

# Combined rotation (ZYX order - yaw, pitch, roll)
combined = rz * ry * rx
println("\nCombined rotation (Z → Y → X):")
display(combined.matrix)
println("\nVisualization of combined rotation:")
result = visualise(combined)
println("Combined rotation is valid: ", is_valid_rotation(combined))

# Extract roll, pitch, yaw from rotation matrix
roll, pitch, yaw = rotation2rpy(combined)
println("\nExtracted roll, pitch, yaw from combined rotation:")
println("roll = ", rad2deg(roll), " degrees")
println("pitch = ", rad2deg(pitch), " degrees")
println("yaw = ", rad2deg(yaw), " degrees")

# Reconstruct rotation from roll, pitch, yaw
reconstructed = rpy2rotation(roll, pitch, yaw)
println("\nReconstruction error (Frobenius norm):")
println(norm(combined.matrix - reconstructed.matrix))