using DualQuatUtils
using LinearAlgebra, StaticArrays

# Create some quaternions for examples
q1 = Quaternion(1.0, 2.0, 3.0, 4.0)
q2 = Quaternion(0.0, 1.0, 1.0, 1.0)
q3 = Quaternion(1.0, 0.0, 0.0, 0.0)
q4 = Quaternion(2.0, 3.0, 4.0, 5.0)

println("Quaternion examples:")
println("q1 = $q1")
println("q2 = $q2")
println("q3 = $q3")
println("q4 = $q4")

# Get scalar parts
println("\nScalar parts:")
println("get_scalar(q1) = ", get_scalar(q1))
println("get_scalar(q2) = ", get_scalar(q2))
println("get_scalar(q3) = ", get_scalar(q3))
println("get_scalar(q4) = ", get_scalar(q4))

# Get vector parts
println("\nVector parts:")
println("get_vector(q1) = ", get_vector(q1))
println("get_vector(q2) = ", get_vector(q2))
println("get_vector(q3) = ", get_vector(q3))
println("get_vector(q4) = ", get_vector(q4))

# Check if quaternions are pure
println("\nIs pure quaternion:")
println("is_pure(q1) = ", is_pure(q1))
println("is_pure(q2) = ", is_pure(q2))
println("is_pure(q3) = ", is_pure(q3))
println("is_pure(q4) = ", is_pure(q4))

# Check if quaternions are real
println("\nIs real quaternion:")
println("is_real(q1) = ", is_real(q1))
println("is_real(q2) = ", is_real(q2))
println("is_real(q3) = ", is_real(q3))
println("is_real(q4) = ", is_real(q4))