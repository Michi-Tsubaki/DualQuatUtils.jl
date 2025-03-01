using DualQuatUtils
using LinearAlgebra, StaticArrays

# Demonstrate dual quaternion basics
println("Dual quaternion basics:")

# Create sample quaternions for real and dual parts
q1 = Quaternion(1.0, 0.0, 0.0, 0.0)  # 1
q2 = Quaternion(0.0, 1.0, 0.0, 0.0)  # i

# Create dual quaternions
dq1 = DualQuaternion(q1, q2)
dq2 = DualQuaternion(q2, q1)

println("\ndq1 = DualQuaternion($(dq1.real), $(dq1.dual))")
println("dq2 = DualQuaternion($(dq2.real), $(dq2.dual))")

# Addition of dual quaternions
dq3 = dq1 + dq2
println("\nAddition: dq1 + dq2 = DualQuaternion($(dq3.real), $(dq3.dual))")

# Scalar multiplication
dq4 = 2.0 * dq1
println("\nScalar multiplication: 2.0 * dq1 = DualQuaternion($(dq4.real), $(dq4.dual))")

# Dual quaternion multiplication
dq5 = dq1 * dq2
println("\nMultiplication: dq1 * dq2 = DualQuaternion($(dq5.real), $(dq5.dual))")

# Conjugations
conj1 = conjugate(dq1)
println("\nConjugate: conjugate(dq1) = DualQuaternion($(conj1.real), $(conj1.dual))")

dual_conj = dual_conjugate(dq1)
println("\nDual conjugate: dual_conjugate(dq1) = DualQuaternion($(dual_conj.real), $(dual_conj.dual))")

full_conj = full_conjugate(dq1)
println("\nFull conjugate: full_conjugate(dq1) = DualQuaternion($(full_conj.real), $(full_conj.dual))")

# Norm and normalization
n = qnorm(dq1)
println("\nNorm: norm(dq1) = $n")

# Create a non-normalized dual quaternion for testing
non_normalized_dq = DualQuaternion(Quaternion(2.0, 1.0, 0.0, 0.0), Quaternion(0.0, 1.0, 1.0, 0.0))
normalized_dq = qnormalize(non_normalized_dq)
println("\nNormalized: normalize(non_normalized_dq) = DualQuaternion($(normalized_dq.real), $(normalized_dq.dual))")

# Identity dual quaternion
identity_dq = DualQuatUtils.one(DualQuaternion)
println("\nIdentity dual quaternion: one(DualQuaternion) = DualQuaternion($(identity_dq.real), $(identity_dq.dual))")