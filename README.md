# DualQuatUtils.jl

[![deps](https://juliahub.com/docs/General/DualQuatUtils/stable/deps.svg)](https://juliahub.com/ui/Packages/General/DualQuatUtils?t=2)
[![version](https://juliahub.com/docs/General/DualQuatUtils/stable/version.svg)](https://juliahub.com/ui/Packages/General/DualQuatUtils)
[![pkgeval](https://juliahub.com/docs/General/DualQuatUtils/stable/pkgeval.svg)](https://juliahub.com/ui/Packages/General/DualQuatUtils)

A Julia package for working with quaternions, rotations, and dual quaternions in 3D space.

There are various conventions for quaternions. This tool assumes right-handed coordinate systems and uses quaternions in 'wxyz' order (scalar part followed by vector parts).

References:
- https://arxiv.org/pdf/1711.02508%20
- https://faculty.sites.iastate.edu/jia/files/inline-files/dual-quaternion.pdf

[日本語版はこちら](#DualQuatUtilsjl-1)

## Features

- Quaternion operations and transformations
- 3D rotations with various representations (matrices, RPY angles, quaternions)
- Dual quaternion operations for rigid body transformations
- Visualization utilities for rotations and transformations

## Installation

```julia
using Pkg
Pkg.add("DualQuatUtils")
```

Or, in the Julia REPL, press `]` to enter the package mode and run:

```
add DualQuatUtils
```

## Quick Start

```julia
using DualQuatUtils
using LinearAlgebra

# Create a quaternion
q = Quaternion(1.0, 0.0, 0.0, 0.0)

# Create a rotation (90 degrees around Z axis)
rot_z = rotation_z(π/2)

# Convert between representations
rot_q = rotation2quat(rot_z)
rot_mat = quat2rotation(rot_q)

# Create a dual quaternion representing a transformation
# (rotate 45 degrees around Z and translate [1,2,3])
θ = π/4
rot_q = Quaternion(cos(θ/2), 0.0, 0.0, sin(θ/2))
translation = [1.0, 2.0, 3.0]
transform = transform_dq(rot_q, translation)

# Convert to homogeneous transformation matrix
H = dq2H(transform)
```

## Key Functions

### Quaternion Operations

- `Quaternion(w, x, y, z)`: Create a quaternion
- `get_scalar(q)`, `get_vector(q)`: Extract scalar and vector parts
- `is_real(q)`, `is_pure(q)`: Check quaternion properties
- `qnorm(q)`, `qnormalize(q)`: Get norm and normalize
- `conjugate(q)`, `inverse(q)`: Get conjugate and inverse

### Rotation Operations

- `rotation_x(angle)`, `rotation_y(angle)`, `rotation_z(angle)`: Create basic rotations
- `rpy2rotation(roll, pitch, yaw)`: Create rotation from RPY angles
- `rotation2rpy(rotation)`: Extract RPY angles from rotation
- `Rodrigues(axis, angle)`: Create rotation using Rodrigues formula
- `quat2rotation(q)`: Convert quaternion to rotation matrix
- `rotation2quat(rot)`: Convert rotation matrix to quaternion

### Dual Quaternion Operations

- `DualQuaternion(real, dual)`: Create a dual quaternion
- `rotation_dq(q)`: Create dual quaternion representing pure rotation
- `translation_dq(v)`: Create dual quaternion representing pure translation
- `transform_dq(q, v)`: Create dual quaternion for rotation followed by translation
- `extract_transform(dq)`: Extract rotation quaternion and translation vector
- `dq2H(dq)`: Convert dual quaternion to homogeneous transformation matrix
- `H2dq(H)`: Convert homogeneous matrix to dual quaternion

## Running Examples

The package includes example files in the `examples/` directory to demonstrate usage:

```bash
# Run a specific example
julia --project=. examples/01_quaternion_basics.jl

# Run all examples
for file in examples/*.jl; do
    echo "Running $file"
    julia --project=. "$file"
done
```

## Running Tests

To run the tests:

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

Or in Julia REPL:

```julia
using Pkg
Pkg.test("DualQuatUtils")
```

---

# DualQuatUtils.jl

3D空間におけるクオータニオン，回転，同次変換行列，双対クオータニオンを扱うためのJuliaパッケージです．
[こちら](https://github.com/Michi-Tsubaki/DualQuatUtils.jl/blob/master/docs/Quaternion_Kinematics.ipynb) のノートブックでクオータニオンや双対クオータニオンについて学べます．（日本語のみ）

## 特徴

- クオータニオンの演算と変換
- 3D回転の様々な表現（行列、RPY角、四元数）
- 剛体変換のための双対クオータニオン演算
- 回転と変換の可視化ユーティリティ

## インストール

```julia
using Pkg
Pkg.add("DualQuatUtils")
```

または、JuliaのREPLで `]` を押してパッケージモードに入り，次を実行する．

```
add DualQuatUtils
```

## クイックスタート

```julia
using DualQuatUtils
using LinearAlgebra

# 四元数の作成
q = Quaternion(1.0, 0.0, 0.0, 0.0)

# 回転の作成（Z軸周りに90度）
rot_z = rotation_z(π/2)

# 表現間の変換
rot_q = rotation2quat(rot_z)
rot_mat = quat2rotation(rot_q)

# 変換を表す双対四元数の作成
# （Z軸周りに45度回転し、[1,2,3]平行移動）
θ = π/4
rot_q = Quaternion(cos(θ/2), 0.0, 0.0, sin(θ/2))
translation = [1.0, 2.0, 3.0]
transform = transform_dq(rot_q, translation)

# 同次変換行列への変換
H = dq2H(transform)
```

## 主要な関数

### 四元数の操作

- `Quaternion(w, x, y, z)`: 四元数の作成
- `get_scalar(q)`, `get_vector(q)`: スカラー部とベクトル部の抽出
- `is_real(q)`, `is_pure(q)`: 四元数の性質チェック
- `qnorm(q)`, `qnormalize(q)`: ノルムの取得と正規化
- `conjugate(q)`, `inverse(q)`: 共役と逆元の取得

### 回転の操作

- `rotation_x(angle)`, `rotation_y(angle)`, `rotation_z(angle)`: 基本回転の作成
- `rpy2rotation(roll, pitch, yaw)`: RPY角から回転行列の作成
- `rotation2rpy(rotation)`: 回転行列からRPY角の抽出
- `Rodrigues(axis, angle)`: ロドリゲスの公式による回転の作成
- `quat2rotation(q)`: 四元数から回転行列への変換
- `rotation2quat(rot)`: 回転行列から四元数への変換

### 双対四元数の操作

- `DualQuaternion(real, dual)`: 双対四元数の作成
- `rotation_dq(q)`: 純粋な回転を表す双対四元数の作成
- `translation_dq(v)`: 純粋な平行移動を表す双対四元数の作成
- `transform_dq(q, v)`: 回転後に平行移動する変換のための双対四元数の作成
- `extract_transform(dq)`: 回転四元数と平行移動ベクトルの抽出
- `dq2H(dq)`: 双対四元数から同次変換行列への変換
- `H2dq(H)`: 同次変換行列から双対四元数への変換

## サンプルの実行

パッケージには`examples/`ディレクトリに使用方法を示すサンプルファイルが含まれています：

```bash
# 特定のサンプルを実行
julia --project=. examples/01_quaternion_basics.jl

# すべてのサンプルを実行
for file in examples/*.jl; do
    echo "Running $file"
    julia --project=. "$file"
done
```

## テストの実行

テストを実行するには，

```bash
julia --project=. -e 'using Pkg; Pkg.test()'
```

または、JuliaのREPLで，

```julia
using Pkg
Pkg.test("DualQuatUtils")
```
