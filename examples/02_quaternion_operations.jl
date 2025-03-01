using DualQuatUtils
using LinearAlgebra, StaticArrays

# Create some quaternions for examples
q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
q2 = Quaternion(0.0, 1.0, 1.0, 1.0)
q3 = Quaternion(1.0, 0.0, 0.0, 0.0)
q4 = Quaternion(2.0, 3.0, 4.0, 5.0)

# Calculate norms
println("Norms:")
println("norm(q1) = ", qnorm(q1))
println("norm(q2) = ", qnorm(q2))
println("norm(q3) = ", qnorm(q3))
println("norm(q4) = ", qnorm(q4))

# Calculate conjugates
println("\nConjugates:")
println("conjugate(q1) = ", conjugate(q1))
println("conjugate(q2) = ", conjugate(q2))
println("conjugate(q3) = ", conjugate(q3))
println("conjugate(q4) = ", conjugate(q4))

# Calculate inverses
println("\nInverses:")
println("inverse(q1) = ", inverse(q1))
println("inverse(q2) = ", inverse(q2))
println("inverse(q3) = ", inverse(q3))
println("inverse(q4) = ", inverse(q4))

# Normalize quaternions
println("\nNormalized quaternions:")
println("normalize(q1) = ", qnormalize(q1))
println("normalize(q2) = ", qnormalize(q2))
println("normalize(q3) = ", qnormalize(q3))
println("normalize(q4) = ", qnormalize(q4))

# Add quaternions
println("\nQuaternion addition:")
println("q1 + q2 = ", q1 + q2)

# Convert quaternion to matrix form
println("\nQuaternion to matrix form:")
println("quat2mat(q1) = ")
display(quat2mat(q1))

# Multiply quaternions
println("\nQuaternion multiplication:")
println("q1 * q2 = ", q1 * q2)