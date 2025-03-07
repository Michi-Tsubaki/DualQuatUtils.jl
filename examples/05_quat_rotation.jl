using DualQuatUtils
using LinearAlgebra, StaticArrays

# Demonstrate Rodrigues rotation formula
println("Rodrigues rotation formula examples:")

# Create a rotation around the X axis using Rodrigues formula
rod_x = Rodrigues([1.0, 0.0, 0.0], pi)
println("\nRotation around X axis by 180 degrees using Rodrigues formula:")
display(rod_x.matrix)

# Compare with standard rotation_x function
rot_x = rotation_x(pi)
println("\nRotation around X axis by 180 degrees using rotation_x function:")
display(rot_x.matrix)

# Check if they're equal
are_equal = isapprox(rod_x.matrix, rot_x.matrix, atol=1e-10)
println("\nAre Rodrigues and standard rotations equal? ", are_equal)

# Create a rotation around an arbitrary axis
axis = [1.0, 1.0, 1.0]  # Diagonal axis
angle = deg2rad(120)    # 120 degrees

# Normalize the axis
axis = axis / norm(axis)
println("\nRotation around normalized axis ", axis, " by 120 degrees:")
rod_arbitrary = Rodrigues(axis, angle)
display(rod_arbitrary.matrix)
println("\nIs this a valid rotation matrix? ", is_valid_rotation(rod_arbitrary))

# Visualize the rotation
println("\nVisualization of rotation around arbitrary axis:")
result = visualise(rod_arbitrary)

# Creating a skew-symmetric matrix
v = [1.0, 2.0, 3.0]
println("\nSkew-symmetric matrix from vector ", v, ":")
skew = skew_sym(v)
display(skew.matrix)

# Verify the properties of skew-symmetric matrices
println("\nVerifying skew-symmetric property (S + S^T = 0):")
sum_matrix = skew.matrix + transpose(skew.matrix)
display(sum_matrix)
println("Maximum absolute value in sum: ", maximum(abs.(sum_matrix)))