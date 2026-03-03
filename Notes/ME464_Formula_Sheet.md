# ME 464 — Kinematics Formula Sheet
### Chapter 2: Rigid Body Motion

---

## 2.1 Rigid Motion

A **rigid body motion** preserves distances and orientations between all points on the body.
The configuration of a rigid body is described by:
- **Rotation:** orientation of the body frame B relative to the spatial frame A
- **Translation:** position of the body frame origin in frame A

A point `p_b` in body frame B is mapped to frame A by:

```
p_a = R_ab * p_b + p_ab
```

Where `R_ab` is the rotation matrix and `p_ab` is the position of B's origin in A.

---

## 2.2 Rotations in R³

### 2.2.1 Properties of Rotation Matrices

A rotation matrix `R ∈ R^(3×3)` has **mutually orthogonal unit columns** `r1, r2, r3`:

```
r_i^T r_j = { 0  if i ≠ j
            { 1  if i = j
```

This gives the **orthogonality condition**:

```
R R^T = R^T R = I
```

Which implies:

```
R^(-1) = R^T
```

And for right-hand coordinate systems:

```
det(R) = +1
```

---

### Special Orthogonal Group SO(3)

The set of all valid 3D rotation matrices forms the group **SO(3)**:

```
SO(3) = { R ∈ R^(3×3) : R R^T = I,  det(R) = +1 }
```

More generally:

```
SO(n) = { R ∈ R^(n×n) : R R^T = I,  det(R) = +1 }
```

- `n = 2` → planar rotation matrices
- `n = 3` → spatial rotation matrices (SO(3) is the rotation group of R³)

---

### Group Properties of SO(3)

| Property       | General                          | Rotation Matrices                        |
|----------------|----------------------------------|------------------------------------------|
| Closure        | g1, g2 ∈ G → g1∘g2 ∈ G          | R1, R2 ∈ SO(3) → R1 R2 ∈ SO(3)          |
| Identity       | g∘e = e∘g = g                    | e = I (3×3 identity)                     |
| Inverse        | g∘g^(-1) = e                     | R^(-1) = R^T,  since R^T R = I           |
| Associativity  | (g1∘g2)∘g3 = g1∘(g2∘g3)         | (R1 R2) R3 = R1 (R2 R3)                  |

**Closure proof:**  `R1 R2 (R1 R2)^T = R1 R2 R2^T R1^T = R1 I R1^T = I`  and  `det(R1 R2) = det(R1) det(R2) = +1` ✓

---

### Rotation Matrix Operations

**Point transformation** (body frame B → spatial frame A):

```
v_a = R_ab * v_b
```

**Composition rule** (chaining rotations):

```
R_ac = R_ab * R_bc
```

`R_ac` maps coordinates from frame C to frame A, by first rotating C→B, then B→A.

---

### 2.2.1b The Hat Operator  (∧)

The **hat operator** maps a 3×1 vector to a 3×3 skew-symmetric matrix:

```
      [ 0   -ω3   ω2 ]
ω̂  = [ ω3   0   -ω1 ]     where  ω = [ω1; ω2; ω3]
      [-ω2   ω1   0  ]
```

Cross product written as matrix multiplication:

```
a × b = â b
```

The **vee operator** (∨) is the inverse of hat:

```
vee(ω̂) = ω     (extracts 3×1 vector from 3×3 skew-symmetric matrix)
```

Extraction indices:  `ω = [ W(3,2);  W(1,3);  W(2,1) ]`

**Skew-symmetry property:**

```
ω̂^T = -ω̂
```

---

### 2.2.2 Exponential Coordinates for Rotations

**Velocity of a rotating point** (rotating about axis ω at unit velocity):

```
q̇(t) = ω × q(t) = ω̂ q(t)
```

**Solution** (matrix exponential):

```
q(t) = e^(ω̂ t) q(0)
```

**Rotation matrix** (rotate about unit axis ω for θ units of time):

```
R(ω, θ) = e^(ω̂ θ)
```

**Matrix exponential definition** (Taylor series):

```
e^A = Σ (1/k!) A^k = I + A + (1/2!) A² + (1/3!) A³ + ...
```

For unit angular velocity:

```
e^(ω̂ t) = I + ω̂ t + (ω̂ t)²/2! + (ω̂ t)³/3! + ...
```

**Lie algebra so(3)** — the space of all 3×3 skew-symmetric matrices:

```
so(3) = { s ∈ R^(n×n) : s^T = -s }
```

**Useful identities** for â ∈ so(3), with ‖a‖ = 1:

```
â²  =  a a^T  −  ‖a‖² I   =   a a^T − I
â³  =  −‖a‖² â             =  −â
```

---

### 2.2.2b Rodrigues' Formula

Closed-form rotation matrix (no infinite series needed):

```
R(ω, θ) = e^(ω̂ θ) = I + sin(θ) ω̂ + (1 − cos(θ)) ω̂²
```

Where `‖ω‖ = 1`.

**Inverse Rodrigues** (given R, find ω and θ):

```
θ = arccos( (trace(R) − 1) / 2 )

ω = (1 / (2 sin(θ))) * vee(R − R^T)     (for θ ≠ 0, π)
```

---

## 2.3 Rigid Motion in R³

### 2.3.1 Homogeneous Transformations

The **Homogeneous Geometric Transformation (HGT)** combines rotation and translation into a single 4×4 matrix:

```
       [ R   p ]
g_ab = [       ]    ∈  SE(3)
       [ 0   1 ]
```

Where `R ∈ SO(3)` and `p ∈ R³`.

**Point transformation** (homogeneous coordinates):

```
[ p_a ]   [ R_ab   p_ab ] [ p_b ]
[     ] = [             ] [     ]
[  1  ]   [  0      1   ] [  1  ]
```

Equivalently:  `p_a = R_ab p_b + p_ab`

**HGT Inverse** (analytic — no matrix inversion needed):

```
         [ R^T    -R^T p ]
g^(-1) = [               ]
         [  0       1    ]
```

**Composition rule:**

```
g_ac = g_ab * g_bc
```

---

### Special Euclidean Group SE(3)

```
SE(3) = { (R, p) : R ∈ SO(3),  p ∈ R³ }
```

SE(3) satisfies the same four group properties as SO(3) (closure, identity, inverse, associativity), extended to include translation.

---

### 2.3.2a Exponential Coordinates for Rigid Motion

#### Revolute Joint (rotation about an axis not through the origin)

Given:
- `ω ∈ R³`, `‖ω‖ = 1` — axis of rotation
- `q ∈ R³` — any point on the axis (in frame A)
- `v = −ω × q = −ω̂ q` — linear component of the twist

**Revolute twist vector** (6×1):

```
ξ = [ v ]  =  [ −ω̂ q ]
    [ ω ]     [   ω   ]
```

**Revolute twist matrix** (4×4, wedge of ξ):

```
      [ ω̂    v  ]   [ ω̂   −ω̂ q ]
ξ̂  = [          ] = [            ]
      [  0   0  ]   [  0    0   ]
```

#### Prismatic Joint (pure translation)

Given:
- `v ∈ R³`, `‖v‖ = 1` — direction of translation

**Prismatic twist vector** (6×1):

```
ξ = [ v ]
    [ 0 ]
```

**Prismatic twist matrix** (4×4):

```
      [ 0   v ]
ξ̂  = [       ]
      [ 0   0 ]
```

---

### Wedge (∧) and Vee (∨) Operators for Twists

**Wedge** maps 6×1 twist coordinates → 4×4 matrix:

```
[ v ]^    [ ω̂   v ]
[   ]  =  [       ]    (4×4)
[ ω ]     [  0  0 ]
```

**Vee** maps 4×4 matrix → 6×1 twist coordinates (inverse of wedge):

```
[ ω̂   v ]^∨  =  [ v ]    (6×1)
[  0  0 ]        [ ω ]
```

---

### Point Motion and Forward Kinematics

**Point motion under a twist** (same-frame mapping):

```
p_a(t) = e^(ξ̂ θ) p_a(0)
```

**Forward kinematics** (body-frame formulation):

```
g_ab(θ) = e^(ξ̂ θ) g_ab(0)

p_a(θ) = g_ab(θ) p_b
```

---

### 2.3.2b Matrix Exponential Formulas

#### Revolute Joint  (‖ω‖ = 1):

```
e^(ξ̂ θ) = [ R,   p  ]
           [ 0,   1  ]
```

Where:

```
R = e^(ω̂ θ)  =  I + sin(θ) ω̂ + (1 − cos(θ)) ω̂²     (Rodrigues)

p = (I − R)(ω × v) + ω ω^T v θ
  = (I − R) ω̂ v                    (simplifies since ω^T v = 0 for revolute)
```

#### Prismatic Joint  (ω = 0):

```
e^(ξ̂ θ) = [ I,   v θ ]
           [ 0,    1  ]
```

---

## 2.4 Rigid Body Velocity

### 2.4.1 Rotational Velocity

For a pure rotation `R_ab(t) ∈ SO(3)`, with `R R^T = I`:

Differentiating gives:  `Ṙ R^T + R Ṙ^T = 0`  →  `Ṙ R^T = −(Ṙ R^T)^T`  (skew-symmetric)

**Instantaneous Spatial Angular Velocity:**

```
ω̂^s_ab  :=  Ṙ_ab R_ab^(-1)  =  Ṙ_ab R_ab^T
```

`ω^s_ab` is the angular velocity as seen from the **spatial (A) frame**.

**Instantaneous Body Angular Velocity:**

```
ω̂^b_ab  :=  R_ab^(-1) Ṙ_ab  =  R_ab^T Ṙ_ab
```

`ω^b_ab` is the angular velocity as seen from the **body (B) frame**.

**Relation between spatial and body angular velocities:**

```
ω̂^b_ab  =  R_ab^(-1) ω̂^s_ab R_ab       (matrix form)

ω^b_ab   =  R_ab^(-1) ω^s_ab             (vector form)
```

**Point velocity using spatial angular velocity:**

```
v_qa(t)  =  ω̂^s_ab q_a(t)  =  ω^s_ab × q_a(t)
```

**Point velocity using body angular velocity:**

```
v_qb(t)  =  ω̂^b_ab q_b  =  ω^b_ab × q_b
```

---

### 2.4.2a Rigid Body Velocity (Spatial and Body)

For a general rigid motion `g_ab(t) ∈ SE(3)`:

**Spatial Velocity** (4×4 matrix form):

```
V̂^s_ab  =  ġ_ab g_ab^(-1)
```

**Body Velocity** (4×4 matrix form):

```
V̂^b_ab  =  g_ab^(-1) ġ_ab
```

**6×1 twist vector form** (using vee operator):

```
V^s_ab  =  vee( V̂^s_ab )  =  veeTwist( ġ_ab g_ab^(-1) )

V^b_ab  =  vee( V̂^b_ab )  =  veeTwist( g_ab^(-1) ġ_ab )
```

Both have the form  `V = [v; ω]`  where `v` (3×1) is the linear component and `ω` (3×1) is the angular component.

---

### 2.4.2b The Adjoint Transformation

The **Adjoint** of a HGT `g = [R, p; 0, 1]` is the 6×6 matrix:

```
         [ R     p̂ R ]
Ad_g  =  [            ]    (6×6)
         [ 0      R  ]
```

Where `p̂ = hat(p)` is the 3×3 skew-symmetric matrix of `p`.

**Transforms spatial velocities between frames:**

```
V^s_ab  =  Ad_{g_ab} V^b_ab
```

**4×4 form of the adjoint relationship:**

```
V̂^s  =  g V̂^b g^(-1)
```

---

### 2.4.3 Rigid Body Velocity using Twists

For a single-joint system `g(θ) = e^(ξ̂ θ) g(0)`:

**Spatial velocity** (does NOT depend on θ for spatial frame formulation):

```
V^s  =  ξ θ̇
```

**Body velocity:**

```
V^b  =  Ad_{g^(-1)} ξ θ̇
```

**Velocity of a body point** `r` at position `r_a` in the spatial frame:

```
ṙ_a  =  v^s + ω^s × r_a
```

Where `v^s = V^s(1:3)` and `ω^s = V^s(4:6)` are the first and last 3 elements of V^s.

In matrix form using V̂^s:

```
[ ṙ_a ]   [ ω̂^s   v^s ] [ r_a ]
[     ] = [             ] [     ]
[  0  ]   [   0    0  ] [  1  ]
```

---

### 2.4.4 Coordinate Transformations for Velocity

For three frames A, B, C with `g_ac = g_ab g_bc`:

**Spatial velocity composition:**

```
V^s_ac  =  V^s_ab  +  Ad_{g_ab} V^s_bc
```

**Twist transformation under a rigid motion** `g ∈ SE(3)`:

```
ξ' = Ad_g ξ         (6×1 vector form)

ξ̂' = g ξ̂ g^(-1)    (4×4 matrix form)
```

---

## 2.5 Wrenches

### 2.5.1 Wrenches

A **wrench** is the dual of a twist — a 6×1 vector combining force and torque:

```
F = [ f ]     f ∈ R³ (force),   τ ∈ R³ (torque/moment)
    [ τ ]
```

**Power** (scalar, frame-independent):

```
P  =  F^T V  =  f · v  +  τ · ω
```

Where `V = [v; ω]` is the twist (velocity).

**Wrench transformation** between frames (using power duality `F^a · V^a = F^b · V^b`):

```
F^b  =  Ad_{g_ab}^T  F^a
```

Expanding with `Ad_{g_ab} = [R, p̂R; 0, R]`:

```
            [  R^T     0  ] [ f^a ]
F^b  =      [             ] [     ]
            [ (p̂R)^T  R^T] [ τ^a ]
```

---

## Summary: MATLAB Functions

| Function              | Operation                                      | In → Out           |
|-----------------------|------------------------------------------------|--------------------|
| `hat_PCM(w)`          | Hat operator (∧)                               | 3×1 → 3×3          |
| `vee_PCM(W)`          | Vee operator (∨) — inverse hat                 | 3×3 → 3×1          |
| `expr_PCM(w, θ)`      | Rotation via Rodrigues: e^(ω̂θ)                | ω, θ → 3×3 R       |
| `twistr_PCM(w, q)`    | Revolute twist: ξ = [−ω̂q; ω]                  | ω, q → 6×1 ξ       |
| `twistp_PCM(v)`       | Prismatic twist: ξ = [v; 0]                    | v → 6×1 ξ          |
| `wedge_PCM(xi)`       | Wedge operator: ξ → ξ̂                          | 6×1 → 4×4          |
| `veeTwist_PCM(Xi)`    | Vee twist operator: ξ̂ → ξ                      | 4×4 → 6×1          |
| `expt_PCM(xi, θ)`     | HGT exponential (revolute): e^(ξ̂θ)            | ξ, θ → 4×4 g       |
| `extp_PCM(xi, θ)`     | HGT exponential (prismatic): [I, vθ; 0,1]      | ξ, θ → 4×4 g       |
| `hgt_PCM(R, p)`       | Build HGT from R and p                         | R, p → 4×4 g       |
| `invHGT_PCM(g)`       | HGT inverse: [R^T, −R^T p; 0,1]               | 4×4 → 4×4          |
| `adjoint_PCM(g)`      | Adjoint: Ad_g = [R, p̂R; 0, R]                 | 4×4 → 6×6          |
| `spatialVel_PCM(g,ġ)` | Spatial velocity: vee(ġ g^(-1))                | g, ġ → 6×1 V^s     |
| `bodyVel_PCM(g,ġ)`    | Body velocity: vee(g^(-1) ġ)                   | g, ġ → 6×1 V^b     |
| `wrenchTransform_PCM(g,F)` | Wrench transform: Ad_g^T F                | g, F^a → 6×1 F^b   |

---

## Key Identities Quick Reference

```
R^(-1)  =  R^T                                    (rotation inverse)

g^(-1)  =  [ R^T,  -R^T p ]                       (HGT inverse)
           [  0,      1   ]

R(ω,θ) = I + sin(θ)ω̂ + (1−cos(θ))ω̂²             (Rodrigues)

ω̂²     = ωω^T − I        (for ‖ω‖ = 1)
ω̂³     = −ω̂              (for ‖ω‖ = 1)

v (revolute) = −ω × q = −ω̂ q

e^(ξ̂θ) = [ R,   (I−R)ω̂v ]   revolute  (‖ω‖ = 1, ω^T v = 0)
          [ 0,      1     ]

e^(ξ̂θ) = [ I,    vθ  ]        prismatic (ω = 0)
          [ 0,     1  ]

ω̂^s  =  Ṙ R^T          ω̂^b  =  R^T Ṙ
V̂^s  =  ġ g^(-1)        V̂^b  =  g^(-1) ġ

V^s  =  Ad_g V^b                                  (spatial ↔ body velocity)
F^b  =  Ad_{g_ab}^T F^a                           (wrench transformation)

V^s_ac  =  V^s_ab + Ad_{g_ab} V^s_bc              (velocity composition)
ξ'      =  Ad_g ξ                                  (twist under rigid motion)
```
