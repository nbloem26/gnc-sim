# Theory manual

The mathematics behind every **shipped** model in gnc-sim: the governing equations,
their derivations or first principles, the integration scheme that advances them, and the
literature each rests on.

This manual is the **"why the equations are what they are"** companion to:

- [`docs/MODELS.md`](MODELS.md) — one page per model: assumptions, the as-implemented
  governing equation, validity limits, references. (The *what it claims / where it stops*.)
- [`docs/VNV_MATRIX.md`](VNV_MATRIX.md) — the **V&V matrix**: each model's claim → its
  evidence (GoogleTest benchmark, golden baseline, analytic check). (The *how we know it's
  right*.)
- [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) — one core / two targets and the per-step loop.
- [`docs/DATA_CONTRACT.md`](DATA_CONTRACT.md) — the JSON/CSV schema the config keys below
  live in.

It deliberately does **not** repeat the credibility tables in MODELS.md / VNV_MATRIX.md —
follow the links there. Here we derive the equations. The model set is exactly the keys
resolved by [`core/src/model/Registry.cpp`](../core/src/model/Registry.cpp); this manual
covers **every** one of them.

For symbol conventions, units, and a worked CLI + Python SDK example, see
[§8 Notation](#8-notation--units) and [§9 Worked example](#9-worked-example-cli--python-sdk).

## Contents

1. [Reference frames & state](#1-reference-frames--state)
2. [Equations of motion & integration](#2-equations-of-motion--integration)
   (`3dof`, `6dof`, `6dof_hifi`; RK4/RK2/Euler)
3. [Environment](#3-environment) (`flat`, `round`, hi-fi EGM/atmosphere/wind)
4. [Aerodynamics](#4-aerodynamics)
5. [Sensors & noise](#5-sensors--noise)
   (`radar`, `ir`, `radar_pheno`, `ir_pheno`; IMU/Allan, RCS/Swerling, CA-CFAR, NETD)
6. [Navigation / estimation](#6-navigation--estimation)
   (`alpha_beta`, `ekf`, `imm`, `jpda`, track-to-track fusion)
7. [Guidance](#7-guidance) (`pronav`, `apn`, `zemzev`, `none`; divert/ACS)
8. [Threats](#8-threats) (`constant`, `weave`, `icbm`, `hgv`, `rv_penaids`)
9. [Many-on-many campaign & WTA](#9-many-on-many-campaign--wta)
10. [Notation & units](#10-notation--units)
11. [Worked example (CLI + Python SDK)](#11-worked-example-cli--python-sdk)
12. [References](#12-references)

---

## 1. Reference frames & state

The per-step state of each entity (`EntityState` in
[`core/include/gncsim/core/Types.hpp`](../core/include/gncsim/core/Types.hpp)) is

$$\mathbf{x} = (\mathbf{r},\ \mathbf{v},\ q,\ \boldsymbol{\omega},\ m),$$

position $\mathbf{r}$, velocity $\mathbf{v}$, attitude unit quaternion $q$, body angular
rate $\boldsymbol{\omega}$, and mass $m$.

**Flat-Earth frame (`env.frame = "flat"`).** A local East-North-Up (ENU) Cartesian frame
anchored at a geodetic origin $(\text{lat}_0, \text{lon}_0, \text{alt}_0)$. Gravity is
$-\hat{\mathbf{z}}$ (Up is $+z$). Altitude is simply $r_z$. This is the default and the one
all noise-free golden runs use.

**Round-Earth frame (`env.frame = "round"`).** State is integrated in an Earth-Centred
Inertial (ECI) frame. Geodetic latitude/longitude/altitude map to ECEF through the WGS-84
ellipsoid, and ECI↔ECEF is a rotation about $\hat{\mathbf{z}}$ by $\omega_\oplus t$ (Earth
rate). Ground impact is the WGS-84 ellipsoid surface. See
[`core/include/gncsim/env/Frames.hpp`](../core/include/gncsim/env/Frames.hpp) and §3.

The two frames are a deliberate abstraction (issue #4): the same dynamics/guidance code runs
in either; only the environment (gravity direction/magnitude, what "altitude" means, what
"ground" means) changes.

### Attitude quaternion

Attitude uses a **Hamilton, scalar-first** unit quaternion $q=(w,x,y,z)$ that rotates a
vector from **body** to **world** frame. For a body-frame vector $\mathbf{u}_b$,

$$\mathbf{u}_w = q\,\mathbf{u}_b\,q^{-1},$$

implemented in [`math/Quaternion.hpp`](../core/include/gncsim/math/Quaternion.hpp)
(`rotate()`). The kinematic relation between attitude and body rate (used by the 6DOF EOM) is

$$\dot q = \tfrac12\, q \otimes \boldsymbol{\omega},$$

with $\boldsymbol{\omega}=(0,\omega_x,\omega_y,\omega_z)$ a pure quaternion. After each RK4
step $q$ is renormalised, $q \leftarrow q/\lVert q\rVert$, to stay on the unit sphere (the
integrator does not preserve the norm exactly).

---

## 2. Equations of motion & integration

### 2.1 Integrators

The core ships generic fixed-step integrators
([`math/Integrators.hpp`](../core/include/gncsim/math/Integrators.hpp)), templated on any
state type with vector-space operators. For $\dot{\mathbf y}=f(t,\mathbf y)$:

- **Euler:** $\mathbf y_{n+1}=\mathbf y_n + \Delta t\, f(t_n,\mathbf y_n)$. $O(\Delta t)$.
- **RK2 (explicit midpoint):**
  $k_1=f(t_n,\mathbf y_n)$, $k_2=f(t_n+\tfrac{\Delta t}{2}, \mathbf y_n+\tfrac{\Delta t}{2}k_1)$,
  $\mathbf y_{n+1}=\mathbf y_n+\Delta t\,k_2$. $O(\Delta t^2)$.
- **RK4 (classical):**
  $$k_1=f(t_n,\mathbf y_n),\quad k_2=f(t_n+\tfrac{\Delta t}{2},\mathbf y_n+\tfrac{\Delta t}{2}k_1),$$
  $$k_3=f(t_n+\tfrac{\Delta t}{2},\mathbf y_n+\tfrac{\Delta t}{2}k_2),\quad k_4=f(t_n+\Delta t,\mathbf y_n+\Delta t\,k_3),$$
  $$\mathbf y_{n+1}=\mathbf y_n+\tfrac{\Delta t}{6}\left(k_1+2k_2+2k_3+k_4\right).$$
  Local truncation error $O(\Delta t^5)$, global $O(\Delta t^4)$.

**RK4 is the default.** Fixed step (no adaptive stepping) is a *determinism* requirement
(AGENTS.md golden rule #2): a fixed $\Delta t$ plus a seeded RNG makes the native and WASM
builds bit-identical. Miss distance is **not** quantised by $\Delta t$ — it is the analytic
closest point of approach interpolated within each step (§7.4), so accuracy does not depend
on step size.

### 2.2 `3dof` — point-mass translational EOM

Three translational degrees of freedom; attitude is not modelled and the guidance
acceleration is applied directly to the point mass:

$$\dot{\mathbf r}=\mathbf v,\qquad \dot{\mathbf v}=\frac{\mathbf F_\text{ext}}{m}+\mathbf g,$$

where $\mathbf F_\text{ext}=\mathbf F_\text{thrust}+\mathbf F_\text{aero}+m\,\mathbf a_\text{cmd}$
is the total external world-frame force (thrust along velocity, aero drag, and the applied
guidance acceleration), and $\mathbf g$ is the gravitational acceleration from the
environment (§3). The Runner assembles $\mathbf F_\text{ext}$ and $\mathbf g$ each step and
calls `step3dof()`
([`dynamics/Dynamics.hpp`](../core/include/gncsim/dynamics/Dynamics.hpp)), which advances
$(\mathbf r,\mathbf v)$ with the selected integrator. No airframe lag, no angle of attack,
no body rates — an instantaneous acceleration response. This is the default model and the
right fidelity for trajectory/guidance studies.

### 2.3 `6dof` — rigid body, scalar inertia

Translation is identical to 3DOF. Attitude adds the rotational EOM with a **scalar** inertia
$I$ (a symmetric / axisymmetric body):

$$\dot{\boldsymbol\omega}=\frac{\mathbf M_\text{body}}{I},\qquad \dot q=\tfrac12\, q\otimes\boldsymbol\omega,$$

with $\mathbf M_\text{body}$ the body moment from the autopilot→fin chain (§7) and $q$
renormalised each step. The scalar inertia drops the gyroscopic cross-coupling term
$\boldsymbol\omega\times(I\boldsymbol\omega)$ — exact for a body with equal principal
inertias, an approximation otherwise. See `step6dof()`.

### 2.4 `6dof_hifi` — rigid body, full inertia tensor

The high-fidelity airframe (issue #35,
[`dynamics/Dynamics6dofHiFi.hpp`](../core/include/gncsim/dynamics/Dynamics6dofHiFi.hpp))
carries the **full inertia tensor** $\mathbf I$ and the gyroscopic coupling, i.e. **Euler's
rigid-body equation**:

$$\mathbf I\,\dot{\boldsymbol\omega}=\mathbf M_\text{body}-\boldsymbol\omega\times(\mathbf I\,\boldsymbol\omega),$$

so $\dot{\boldsymbol\omega}=\mathbf I^{-1}\!\left[\mathbf M_\text{body}-\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)\right]$,
integrated by RK4 with the same quaternion kinematics. In the torque-free limit
($\mathbf M_\text{body}=0$) the scheme conserves rotational kinetic energy
$T=\tfrac12\boldsymbol\omega^{\!\top}\mathbf I\boldsymbol\omega$ and angular momentum
$\mathbf I\boldsymbol\omega$ to integrator tolerance — a benchmarked check
(`sixdof_test`). The aero-moment ($C_n, C_m$) tables and actuator (fin rate/travel)
dynamics are assembled by the Runner; this model owns the tensor and the coupled integration.

**Reference.** Stevens & Lewis, *Aircraft Control and Simulation*, ch. 1–2; Euler's
equations for a rigid body.

---

## 3. Environment

### 3.1 Gravity

**`flat` frame.** Local-vertical gravity is $\mathbf g=-g(h)\,\hat{\mathbf z}$. With
`altitude_dependent_g` the magnitude falls off inverse-square about the Earth radius
$R_\oplus$:

$$g(h)=g_0\left(\frac{R_\oplus}{R_\oplus+h}\right)^2,$$

otherwise it is the constant $g_0=9.80665\ \text{m/s}^2$
([`env/Environment.hpp`](../core/include/gncsim/env/Environment.hpp), `GravityModel`).

**`round` frame.** Central two-body gravity in ECI,

$$\mathbf g=-\frac{\mu}{\lVert\mathbf r\rVert^3}\,\mathbf r,$$

$\mu=GM_\oplus$, optionally plus the **J2** oblateness perturbation.

**`round` hi-fi — zonal geopotential through J4** (issue #41,
[`env/EnvFidelity.hpp`](../core/include/gncsim/env/EnvFidelity.hpp)). The acceleration is the
gradient of the truncated **zonal** geopotential. The potential, with $\phi$ the
geocentric latitude, $R_\oplus$ the equatorial radius, and $P_n$ the Legendre polynomials, is

$$U(\mathbf r)=\frac{\mu}{r}\left[1-\sum_{n=2}^{4} J_n\left(\frac{R_\oplus}{r}\right)^{n}P_n(\sin\phi)\right],$$

with $J_2,J_3,J_4$ the zonal harmonic coefficients. $\mathbf g=\nabla U$ is evaluated in
closed form per term. With $J_3=J_4=0$ this reduces **bit-for-bit** to central + J2 (a
regression guard). Zonal-only: no tesseral/sectoral terms and no third-body.

**Reference.** Vallado, *Fundamentals of Astrodynamics and Applications* (geopotential,
ECI/ECEF, J2).

### 3.2 Atmosphere

**USSA76 (`flat` and base `round`).** The **U.S. Standard Atmosphere 1976** is a piecewise
model over geopotential-altitude layers. Within a layer with base values
$(T_b,p_b,h_b)$ and lapse rate $L_b=\mathrm{d}T/\mathrm{d}h$:

$$T(h)=T_b+L_b\,(h-h_b),$$
$$p(h)=\begin{cases}p_b\left(\dfrac{T_b}{T}\right)^{g_0 M/(R^* L_b)} & L_b\neq 0,\\[1.4ex]p_b\,\exp\!\left(-\dfrac{g_0 M (h-h_b)}{R^* T_b}\right) & L_b=0,\end{cases}$$

density from the ideal-gas law $\rho=pM/(R^*T)$, and speed of sound
$a=\sqrt{\gamma R^* T/M}$ ([`atmosphereUSSA76`](../core/include/gncsim/env/Environment.hpp)).
Valid ~0–86 km; clamps outside. The Python twin `postproc/gncpost/atmosphere.py` validates
the same layers (V&V matrix → `atmosphere.py`).

**Extended atmosphere + wind (`round` hi-fi).** Above the USSA76 ceiling the density extends
exponentially/layered, and an opt-in parameterised wind profile is added. The handover at the
USSA76 ceiling is continuous (benchmarked). No F10.7/Ap/diurnal terms (full NRLMSISE-00 is
future work).

### 3.3 Rotating ECEF (hi-fi)

The hi-fi round path can propagate directly in the rotating ECEF frame, adding the
fictitious accelerations explicitly:

$$\mathbf a_\text{ECEF}=\mathbf a_\text{inertial}-2\,\boldsymbol\omega_\oplus\times\mathbf v-\boldsymbol\omega_\oplus\times(\boldsymbol\omega_\oplus\times\mathbf r),$$

the Coriolis and centrifugal terms. ECEF↔ECI agree to $<1$ m / $<5$ cm/s round-trip in the
benchmark (`env_fidelity_test`).

---

## 4. Aerodynamics

[`aero/Aero.hpp`](../core/include/gncsim/aero/Aero.hpp). The drag force opposes the
air-relative velocity:

$$\mathbf F_\text{drag}=-\tfrac12\,\rho\,V^2\,C_d(M)\,A\,\hat{\mathbf v},$$

with dynamic pressure $\bar q=\tfrac12\rho V^2$, reference area $A$, and a **Mach-dependent**
drag coefficient $C_d(M)$ (the transonic rise is modelled). $\rho$ and the speed of sound
(for $M=V/a$) come from the atmosphere (§3.2).

For **6DOF** the airframe also develops a **normal force** from angle of attack $\alpha$,
$\mathbf F_N=\tfrac12\rho V^2 C_{N\alpha}\,\alpha\,A\,\hat{\mathbf n}$ perpendicular to the
velocity in the pitch plane — this is the aerodynamic lever the autopilot uses to turn the
body. The hi-fi model additionally consumes $C_n/C_m$ moment tables (§2.4).

**Reference.** Standard missile/aerodynamics drag-polar treatment; Zarchan ch. 2 for the
point-mass drag term.

---

## 5. Sensors & noise

### 5.1 IMU error model & Allan variance

The strapdown IMU ([`sensors/Sensors.hpp`](../core/include/gncsim/sensors/Sensors.hpp))
corrupts true specific force and angular rate with the error terms recovered from an
**Allan-variance** characterisation of synthetic data (`sensors/`, written to
`configs/sensor_params.json`). The measurement is

$$\tilde{\mathbf u}=(1+s)\,\mathbf u+\mathbf b(t)+\mathbf n_w,$$

scale-factor error $s$, bias $\mathbf b(t)$, and white noise $\mathbf n_w$. The bias is a
first-order **Gauss-Markov** process plus a random walk,

$$\dot{\mathbf b}=-\frac{1}{\tau}\mathbf b+\mathbf w,$$

so the IMU holds bias state between calls (construct once per run). The Allan deviation
$\sigma_A(\tau)$ separates these by their slope versus averaging time $\tau$:
**angle/velocity random walk** $\propto\tau^{-1/2}$, **bias instability** flat (the
$\tau^0$ floor), **rate random walk** $\propto\tau^{+1/2}$. The Python fit recovers each
coefficient from the log-log slope; `postproc/tests` asserts slope recovery.

**Reference.** IEEE Std 952 (Allan-variance inertial-sensor characterisation); Bar-Shalom §6.

### 5.2 Track-sensor measurement models

A fixed-position track sensor measures the geometry to the target with independent zero-mean
Gaussian noise (drawn from the run RNG by Box-Muller in a fixed channel order, preserving
native↔WASM determinism). With $\boldsymbol\Delta=\mathbf r_\text{tgt}-\mathbf r_\text{sensor}$:

**`radar` (full state):**
$$\text{az}=\operatorname{atan2}(\Delta_y,\Delta_x)+n_\text{az},\quad \text{el}=\operatorname{atan2}\!\big(\Delta_z,\sqrt{\Delta_x^2+\Delta_y^2}\big)+n_\text{el},$$
$$\text{range}=\lVert\boldsymbol\Delta\rVert+n_r,\quad \dot r=\frac{\boldsymbol\Delta\cdot\mathbf v}{\lVert\boldsymbol\Delta\rVert}+n_{\dot r}.$$

**`ir` (angles-only):** the az/el pair only — no range or range-rate, so range is only
observable through motion/fusion (bearings-only observability).

**Reference.** Skolnik, *Introduction to Radar Systems*; Bar-Shalom §3 (Cartesian↔polar geometry).

### 5.3 Radar phenomenology `radar_pheno` (signal → detection)

A detection front-end (issue #39,
[`sensors/Phenomenology.hpp`](../core/include/gncsim/sensors/Phenomenology.hpp)) decides
whether each look yields a measurement at all.

**SNR — monostatic range equation,** anchored at a reference SNR for a reference RCS at a
reference range and scaling as $\sigma/R^4$, with clutter (CNR) and barrage-noise jamming
(JNR) raising the noise floor:

$$\text{SNR}=10^{\,\text{SNR}_\text{ref,dB}/10}\cdot\frac{\sigma}{\sigma_\text{ref}}\cdot\left(\frac{R_\text{ref}}{R}\right)^{4}\cdot\frac{1}{1+\text{CNR}+\text{JNR}}.$$

**RCS fluctuation — Swerling cases.** The instantaneous RCS $\sigma$ fluctuates per the
configured Swerling case: 0 non-fluctuating; **I/II** exponential (χ², 2 dof),
$\sigma=\bar\sigma\,(-\ln U)$; **III/IV** (χ², 4 dof),
$\sigma=\bar\sigma\cdot\tfrac12(-\ln U_1-\ln U_2)$, with $U,U_i\sim\mathcal U(0,1)$.

**CA-CFAR detection.** Cell-averaging constant-false-alarm-rate detection over $N$ reference
cells. For a design false-alarm probability $P_\text{fa}$, the threshold multiplier
(Gandhi & Kassam) is

$$\alpha=N\!\left(P_\text{fa}^{-1/N}-1\right),$$

and the single-look detection probability in a homogeneous Rayleigh background is

$$P_d=\left(1+\frac{\alpha}{N\,(1+\text{SNR})}\right)^{-N},$$

which collapses to $P_\text{fa}$ at $\text{SNR}=0$ (noise only). A Bernoulli draw $U<P_d$
decides the look; on a hit, the §5.2 az/el/range/range-rate measurement is produced and
fused, on a miss the tracker coasts. RNG order per step: RCS draw → CFAR Bernoulli → (on a
hit) the Gaussian channel noise — fixed, so parity holds.

**Reference.** Skolnik (range equation, Swerling); Gandhi & Kassam, "Analysis of CFAR
processors in nonhomogeneous background," *IEEE T-AES* 24(4), 1988; Richards, *Fundamentals
of Radar Signal Processing*.

### 5.4 IR phenomenology `ir_pheno` (NETD + atmosphere → detection)

An angles-only sensor whose apparent thermal **contrast** falls inverse-square with range and
is attenuated by **Beer-Lambert** atmospheric transmission $e^{-\beta R}$:

$$C(R)=\Delta T_\text{ref}\left(\frac{R_\text{ref}}{R}\right)^{2}e^{-\beta\,(R-R_\text{ref})},\qquad \text{SNR}=\frac{C(R)}{\text{NETD}}.$$

The same CA-CFAR detector (§5.3) gates the look. On a detection the az/el angular noise is
tied to the signal — a stronger SNR gives a tighter centroid:

$$\sigma_\text{angle}=\frac{\theta_\text{res}}{k\,\sqrt{\text{SNR}}}\quad(\text{floored}).$$

The IR SNR is deterministic given geometry, so the only random draw before a hit is the CFAR
Bernoulli.

**Reference.** Hudson, *Infrared System Engineering* (NETD, contrast); Beer-Lambert
extinction; Gandhi & Kassam (CFAR).

---

## 6. Navigation / estimation

### 6.1 `alpha_beta` — fixed-gain tracker

A steady-state two-state filter on a reconstructed relative-position measurement $\mathbf z$,
assuming constant velocity between updates ([`gnc/Gnc.hpp`](../core/include/gncsim/gnc/Gnc.hpp)):

$$\hat{\mathbf x}\leftarrow\hat{\mathbf x}+\alpha\,(\mathbf z-\hat{\mathbf x}),\qquad \hat{\mathbf v}\leftarrow\hat{\mathbf v}+\frac{\beta}{\Delta t}\,(\mathbf z-\hat{\mathbf x}).$$

Cheap, robust, no covariance — suboptimal and lags a maneuvering target; adequate for the
noise-free default. **Reference.** Bar-Shalom §6.

### 6.2 `ekf` — relative-state Extended Kalman Filter

A **continuous-white-noise-acceleration (CWNA)** process model with the vehicle acceleration
as a known control input, and the **nonlinear** az/el/range measurement linearised about the
estimate ([`gnc/Ekf.hpp`](../core/include/gncsim/gnc/Ekf.hpp)):

$$\textbf{Predict:}\quad \hat{\mathbf x}^-=\mathbf F\hat{\mathbf x}+\mathbf G\,\mathbf a_\text{veh},\qquad \mathbf P^-=\mathbf F\mathbf P\mathbf F^{\!\top}+\mathbf Q(q_\text{psd}),$$
$$\textbf{Update:}\quad \mathbf y=\mathbf z-h(\hat{\mathbf x}^-),\quad \mathbf S=\mathbf H\mathbf P^-\mathbf H^{\!\top}+\mathbf R,\quad \mathbf K=\mathbf P^-\mathbf H^{\!\top}\mathbf S^{-1},$$
$$\hat{\mathbf x}^+=\hat{\mathbf x}^-+\mathbf K\mathbf y,\qquad \mathbf P^+=(\mathbf I-\mathbf K\mathbf H)\mathbf P^-,$$

$\mathbf H=\partial h/\partial\mathbf x$ the measurement Jacobian. It reports the
**normalised innovation squared** $\text{NIS}=\mathbf y^{\!\top}\mathbf S^{-1}\mathbf y$; for
a consistent filter $\mathbb E[\text{NIS}]\approx\dim(\mathbf z)$ (the χ² consistency check
in `ekf_test`). Single-model — lags a hard maneuver (the IMM addresses this); range
observability is weak for angles-only IR. **Reference.** Bar-Shalom §6, §10.

### 6.3 `imm` — Interacting Multiple Model filter

A bank of two EKFs — a constant-velocity (CV) mode and a higher-process-noise maneuver mode
— mixed by mode probability under a Markov transition matrix
([`gnc/Imm.hpp`](../core/include/gncsim/gnc/Imm.hpp)). Each cycle:

1. **Mixing** — form per-mode mixed initial conditions from the mode probabilities $\mu_i$
   and transition probabilities $\pi_{ij}$.
2. **Mode-matched filtering** — run each mode's EKF predict/update (§6.2), recording the
   measurement likelihood $\Lambda_j=\mathcal N(\mathbf y_j;0,\mathbf S_j)$.
3. **Mode-probability update** —
   $\mu_j\propto\Lambda_j\sum_i\pi_{ij}\mu_i$, normalised.
4. **Combination** — the output estimate is the probability-weighted mixture of the mode
   estimates; the reported NIS is the mode-weighted combined NIS.

Best when the target alternates benign/maneuvering flight; two modes only. **Reference.**
Bar-Shalom §11; Blom & Bar-Shalom (1988).

### 6.4 `jpda` — Joint Probabilistic Data Association + lifecycle + fusion

For the multi-object scene (lethal target + closely-spaced objects + Poisson clutter), each
sensor look is a **set** of detections; the associator decides which (if any) updates the
track ([`gnc/DataAssociation.hpp`](../core/include/gncsim/gnc/DataAssociation.hpp),
[`gnc/TargetTrackEkf.hpp`](../core/include/gncsim/gnc/TargetTrackEkf.hpp)).

**Validation gate.** Detection $j$ is gated if its NIS is within the χ² threshold
$\gamma$ (`gate_chi2`):

$$d_j^2=\mathbf y_j^{\!\top}\mathbf S^{-1}\mathbf y_j\le\gamma,\qquad \mathbf y_j=\mathbf z_j-h(\hat{\mathbf x}).$$

**PDA / JPDA-marginal association weights.** For gated detections, with detection
probability $P_D$ and clutter density $\lambda$,

$$\beta_j\propto P_D\,\mathcal N(\mathbf y_j;0,\mathbf S),\qquad \beta_0\propto(1-P_D)\,\lambda,\qquad \beta_0+\sum_j\beta_j=1,$$

$\beta_0$ the no-detection / all-clutter hypothesis (for a single confirmed track the JPDA
marginals reduce to these PDA weights).

**PDA update.** With the combined innovation $\bar{\mathbf y}=\sum_j\beta_j\mathbf y_j$ and
$\mathbf K=\mathbf P^-\mathbf H^{\!\top}\mathbf S^{-1}$:

$$\hat{\mathbf x}^+=\hat{\mathbf x}^-+\mathbf K\bar{\mathbf y},$$
$$\mathbf P^+=\mathbf P^- -(1-\beta_0)\,\mathbf K\mathbf S\mathbf K^{\!\top}+\mathbf K\!\left(\textstyle\sum_j\beta_j\mathbf y_j\mathbf y_j^{\!\top}-\bar{\mathbf y}\bar{\mathbf y}^{\!\top}\right)\!\mathbf K^{\!\top},$$

the last "spread of the innovations" term inflating $\mathbf P$ for the association
uncertainty.

**Lifecycle (M-of-N).** A track confirms once associated on $\ge$ `confirm_m` of the last
`confirm_n` looks and deletes after `delete_misses` consecutive misses.

**Track-to-track fusion — Covariance Intersection.** Per-sensor tracks are fused with a
guaranteed-consistent rule that needs no cross-correlation knowledge:

$$\mathbf P_f^{-1}=w\,\mathbf P_a^{-1}+(1-w)\,\mathbf P_b^{-1},\qquad \mathbf x_f=\mathbf P_f\!\left(w\,\mathbf P_a^{-1}\mathbf x_a+(1-w)\,\mathbf P_b^{-1}\mathbf x_b\right),$$

$w\in(0,1)$ chosen to minimise $\operatorname{tr}(\mathbf P_f)$ by golden-section search
(ties → $w=0.5$). CI does not beat a single sensor for two *identical* inputs (the price of
robustness); the gain is from **complementary** information.

**Reference.** Bar-Shalom & Fortmann, *Tracking and Data Association*; Blackman & Popoli,
*Design and Analysis of Modern Tracking Systems* (M-of-N, MHT); Julier & Uhlmann
(Covariance Intersection).

### 6.5 Discrimination

The feature-based decoy/CSO discriminator
([`gnc/Discriminator.hpp`](../core/include/gncsim/gnc/Discriminator.hpp), issue #6) scores
objects on a kinematic/feature margin (e.g. the ballistic-coefficient deceleration cue from
`rv_penaids`, §8.5) to select the lethal object. It is **complementary** to the kinematic
JPDA gating: JPDA decides *which detection updates a track*; discrimination decides *which
track is the threat*. See MODELS.md for the scoring rule and `Discriminator.hpp`.

---

## 7. Guidance

All laws issue a commanded acceleration $\mathbf a_\text{cmd}$, magnitude-limited to the
vehicle's lateral acceleration cap `max_accel` (or the divert authority under ACS, §7.5).
See [`gnc/Gnc.hpp`](../core/include/gncsim/gnc/Gnc.hpp) and
[`model/Registry.hpp`](../core/include/gncsim/model/Registry.hpp). The navigator supplies the
line-of-sight unit vector $\hat{\mathbf u}_\text{LOS}$, its rotation rate $\boldsymbol\Omega$,
the closing speed $V_c$, and (for APN/ZEM-ZEV) a target-acceleration estimate $\mathbf a_T$.

### 7.1 `pronav` — Proportional Navigation

Drive the commanded acceleration proportional to the inertial LOS rate, perpendicular to the
LOS:

$$\mathbf a_\text{cmd}=N\,V_c\,(\boldsymbol\Omega\times\hat{\mathbf u}_\text{LOS}),$$

$N$ the dimensionless navigation constant (`guidance.nav_constant`, typically 3–5).
**Rationale.** On a collision course the LOS direction is fixed, so $\boldsymbol\Omega=0$;
PN nulls any LOS rotation, steering the interceptor back onto the collision triangle. Optimal
against a non-maneuvering target on a near-collision course; once saturated at `max_accel`,
miss grows. Commands only while closing ($V_c>0$). **Reference.** Zarchan ch. 2.

### 7.2 `apn` — Augmented Proportional Navigation

PN plus a feed-forward of the (estimated) target acceleration perpendicular to the LOS:

$$\mathbf a_\text{cmd}=N\,V_c\,(\boldsymbol\Omega\times\hat{\mathbf u}_\text{LOS})+\frac{N}{2}\,\mathbf a_{T,\perp},$$

the $N/2$ augmentation gain being the optimal feed-forward for a constant-acceleration
target. Only helps when the target-accel estimate is good. **Reference.** Zarchan ch. 4.

### 7.3 `zemzev` — optimal ZEM/ZEV (predictive) guidance

The closed-form linear-quadratic optimal-control intercept law (minimum control energy to
null the predicted miss), using relative position $\mathbf r$, relative velocity $\mathbf v$,
target accel estimate $\mathbf a_T$, and time-to-go $t_\text{go}=\text{range}/V_c$ (floored
at `tgo_floor_s`):

$$\mathbf a_\text{cmd}=\frac{N_\text{zem}}{t_\text{go}^2}\,\textbf{ZEM}+w(\text{range})\,\frac{N_\text{zev}}{t_\text{go}}\,\textbf{ZEV},$$

**Zero-effort miss** (predicted relative position at intercept if neither side accelerates
further) and **zero-effort velocity error**:

$$\textbf{ZEM}=\mathbf r+\mathbf v\,t_\text{go}+\tfrac12\,\mathbf a_T\,t_\text{go}^2,\qquad \textbf{ZEV}=(\mathbf v+\mathbf a_T\,t_\text{go})-\mathbf v_\text{des}.$$

With $N_\text{zem}=3$ the ZEM term is the energy-optimal terminal law; the
$\tfrac12\mathbf a_T t_\text{go}^2$ folds in the target maneuver (a constant-accel target is
intercepted with zero steady-state miss). The ZEV term shapes the **midcourse** trajectory
toward a desired closing geometry; set $N_\text{zev}=0$ for pure terminal homing.

**Midcourse → terminal handover.** The weight $w(\text{range})$ is $1$ in midcourse (ZEV
active), ramps **linearly to 0** across the `handover_blend_m` band above the switch range,
and $0$ inside it (pure terminal ZEM). $w$ is continuous in range, so the command has no
discontinuity at the switch (`optimal_guidance_test::ZemZev.HandoverIsContinuous`).
**Reference.** Zarchan ch. 8; Ben-Asher & Yaesh, *Advances in Missile Guidance Theory*.

### 7.4 `none` — unguided

$\mathbf a_\text{cmd}\equiv 0$ — a deliberate null for ballistic rounds/projectiles. Their
trajectory accuracy is then entirely the dynamics/env/aero models' responsibility.

### 7.5 Divert / ACS actuation

For exo-atmospheric interceptors (`guidance.divert.enabled`), the realised guidance
acceleration is a reaction-control-system divert command **hard magnitude-limited to
`divert_limit_mps2`** (thruster authority), distinct from the aero `max_accel` lift cap.
Usable with any law, exercised with `zemzev` for the exo case.

### 7.6 Miss distance — analytic CPA

The simulation does **not** stop at intercept. Miss distance is the analytic closest point of
approach (CPA) interpolated within each step (`Runner.cpp`): over step $[t_n,t_{n+1}]$ the
relative position is linearised, $\mathbf d(t)=\mathbf d_n+(t-t_n)\dot{\mathbf d}_n$, and the
within-step minimum range is taken at $t^\star=-\,\mathbf d_n\!\cdot\!\dot{\mathbf d}_n/\lVert\dot{\mathbf d}_n\rVert^2$
(clamped to the step). The global minimum over the flight is the miss; intercept is
`best_range < kLethalRadius` (3 m). This is **sub-$\Delta t$ accurate** and independent of
step size — a golden rule (AGENTS.md #3), so don't "simplify" it to nearest-sample distance.

---

## 8. Threats (target maneuver)

Selected by `target.maneuver`. The maneuver supplies the target's applied acceleration each
step ([`model/Threats.hpp`](../core/include/gncsim/model/Threats.hpp)).

### 8.1 `constant`

$\mathbf a_\text{target}=0$ — the benign, ballistic/constant-velocity baseline (PN-optimal).

### 8.2 `weave`

A constant-amplitude sinusoidal lateral acceleration, horizontal and perpendicular to the
ground track:

$$\mathbf a_\text{target}=\hat{\mathbf p}\cdot g_\text{man}\,g_0\,\sin(2\pi f t+\varphi),$$

$\hat{\mathbf p}$ the in-plane unit vector perpendicular to the horizontal velocity,
$g_\text{man}=$ `maneuver_g`, $f=$ `maneuver_freq`, $\varphi=$ `maneuver_phase_deg`,
$g_0=9.80665$. The canonical evasive maneuver driving the APN/IMM benchmarks. **Reference.**
Zarchan ch. 4.

### 8.3 `icbm` — multi-stage boost

A serial stack of boosting stages plus a payload. During stage $k$'s burn, thrust acts along
the velocity $\hat{\mathbf v}$ with a linear (constant mass-flow) burn, under constant gravity
$g_0$:

$$\mathbf a=\hat{\mathbf v}\,\frac{T_k}{m(t)}-g_0\,\hat{\mathbf z},\qquad m(t)=M_0-\sum(\text{jettisoned dry+propellant})-(\text{propellant burned this stage}),$$

with $M_0=m_\text{payload}+\sum_k(m_{\text{dry},k}+m_{\text{prop},k})$. At burn-out the spent
stage's dry mass is jettisoned (the discrete staging mass drop) and the next ignites; after
the last stage the threat coasts ($\mathbf a=-g_0\hat{\mathbf z}$). Reproduces the lofted ICBM
arc (apogee $\sim10^3$ km) and the staging drops. Point-mass, flat-Earth, no threat drag.
**Reference.** Tsiolkovsky/serial staging; Wertz, *Orbit & Constellation Design*.

### 8.4 `hgv` — hypersonic glide vehicle

A lifting reentry body. Below `pull_up_alt_m` it develops lift $\propto(L/D)\cdot\text{drag}$
perpendicular to velocity (toward $+z$); drag opposes velocity. The atmosphere is a
self-contained exponential profile $\rho=\rho_0 e^{-h/H}$, drag deceleration
$\bar q/\beta=\tfrac12\rho V^2/\beta$ with ballistic coefficient $\beta$:

$$\mathbf a=-g_0\hat{\mathbf z}-\hat{\mathbf v}\,\frac{\tfrac12\rho V^2}{\beta}+\hat{\mathbf n}\,(L/D)\,\frac{\tfrac12\rho V^2}{\beta},$$

the lift term gated off above `pull_up_alt_m`. Reproduces the **skip-glide oscillation**
(repeated pull-ups) and monotone $L/D\to$ downrange. Vertical-plane point-mass; no banking,
no thermodynamics. **Reference.** Sänger–Bredt skip-glide; Eggers boost-glide; Allen–Eggers
reentry.

### 8.5 `rv_penaids` — reentry vehicle + penetration aids

A heavy ballistic RV ($\mathbf a=-g_0\hat{\mathbf z}$) and, after `deploy_time_s`, a
deterministic radial fan of $N$ lighter penaids/decoys. Penaid $i$ gets a velocity offset in
the plane perpendicular to the RV velocity,

$$\Delta\mathbf v_i=(\hat{\mathbf e}_1\cos\theta_i+\hat{\mathbf e}_2\sin\theta_i)\,\text{deploy\_dv},\qquad \theta_i=\frac{2\pi i}{N},$$

and propagates with an extra atmospheric deceleration $\mathbf a=-g_0\hat{\mathbf z}-\hat{\mathbf v}\,a_\text{penaid}$
(lower ballistic coefficient sheds speed faster than the heavy RV). That deceleration
difference is the kinematic discrimination cue (§6.5). Deterministic (no RNG); the noisy
multi-feature signature model lives in the `decoys` block (issue #6). **Reference.**
Reentry-vehicle penaids / ballistic-coefficient separation.

---

## 9. Many-on-many campaign & WTA

> **Scenario-level orchestration, not a Registry model.** The campaign
> ([`scenario/ManyOnMany.hpp`](../core/include/gncsim/scenario/ManyOnMany.hpp), issue #45)
> reuses the *same* per-engagement physics — every interceptor×threat pairing is scored by
> running the same deterministic `runSimulation()`. Nothing here changes a single engagement;
> the default path is byte-identical. It is CLI/SDK-only (not in the WASM entry).

**Single-shot kill probability** from the analytic CPA miss, via a Gaussian lethality
(Carleton) damage function:

$$P_\text{ssk}=p_{k,\max}\,\exp\!\left(-\tfrac12\left(\frac{\text{miss}}{\sigma_{pk}}\right)^2\right).$$

**Cumulative kill** of $k$ independent shots on one threat:

$$P_k=1-\prod_{i=1}^{k}\left(1-P_{\text{ssk},i}\right).$$

**Weapon-target assignment (WTA).** Maximise expected kills over the $P_\text{ssk}$ matrix:

- *Greedy* — repeatedly commit the highest-$P_\text{ssk}$ remaining (weapon, threat) pairing,
  one weapon per threat per wave; ties → lowest index (deterministic).
- *Auction* — a Bertsekas ascending-price auction over the same matrix, reaching the same
  optimal one-to-one matching on a clean matrix.

**Doctrines.** **Salvo** commits `shots_per_threat` weapons to each threat at once;
**shoot-look-shoot** fires one weapon per surviving threat per wave, assesses (a threat is
assessed killed once $P_k\ge0.5$), and re-engages survivors for up to `max_waves`; **raid**
defends $M$ threats with a finite inventory, surplus weapons backing up threats round-robin
via repeated WTA passes.

**Rollup.** `leakers` = threats with $P_k<0.5$; expected leakage $\sum_t(1-P_{k,t})$;
$P(\text{raid annihilation})=\prod_t P_{k,t}$. With `num_trials > 1` each committed shot's
kill is a seeded Bernoulli draw (project `Rng`, no `std` distribution → native↔WASM
identical), giving Monte Carlo mean leakage and $P(\text{annihilation})$.

Threats are independent for the campaign rollup; there is no fuzing/fragmentation model and
no inter-threat coupling. **Reference.** Hosein & Athans (WTA); Bertsekas (auction
algorithms); Driels, *Weaponeering* (Carleton/Gaussian lethality); Wagner et al., *Naval
Operations Analysis* (shoot-look-shoot).

---

## 10. Notation & units

| Symbol | Meaning | Units |
|---|---|---|
| $\mathbf r,\mathbf v,\mathbf a$ | position / velocity / acceleration | m, m/s, m/s² |
| $q,\boldsymbol\omega,\mathbf I$ | attitude quaternion / body rate / inertia tensor | –, rad/s, kg·m² |
| $\mathbf g$ | gravitational acceleration | m/s² |
| $\rho,\bar q,M,a$ | air density / dynamic pressure / Mach / speed of sound | kg/m³, Pa, –, m/s |
| $\hat{\mathbf u}_\text{LOS},\boldsymbol\Omega,V_c$ | LOS unit vector / LOS rate / closing speed | –, rad/s, m/s |
| $N$ | navigation constant | – (dimensionless) |
| $t_\text{go}$ | time-to-go | s |
| $\mathbf P,\mathbf S,\mathbf K$ | state covariance / innovation covariance / Kalman gain | — |
| $P_d,P_\text{fa},P_\text{ssk},P_k$ | detection / false-alarm / single-shot / cumulative kill | – |
| $\sigma$ (radar) | radar cross section | m² |
| $\beta$ | ballistic coefficient | kg/m² |

Code follows **units-in-variable-names** (AGENTS.md): physical quantities carry an SI suffix
(`range_m`, `los_rate_radps`, `accel_cmd_mps2`, `divert_limit_mps2`). Data-contract keys
(config JSON / CSV channel names) are the cross-language schema and are versioned separately.

---

## 11. Worked example (CLI + Python SDK)

A minimal end-to-end run of the default homing engagement, both ways. (Both call the **same**
pure C++ core, so the metrics match bit-for-bit.)

### CLI (native → CSV + manifest)

```bash
./scripts/build-native.sh                                   # cmake Release + CTest
./build-native/apps/cli/gncsim \
    --config configs/homing_3dof.json --out runs/run_001
# writes runs/run_001/{vehicle,target,gnc,sensors,track,discrim}.csv + manifest.json
```

`manifest.json` carries the scalar outcome (`miss_distance`, `intercept`, `intercept_time`,
`seed`, `git_sha`). The per-channel CSVs follow [DATA_CONTRACT.md](DATA_CONTRACT.md). Swap
`--config` for any preset in `configs/` (e.g. `homing_zemzev.json`, `threat_hgv.json`,
`track_fused.json`, `many_on_many_raid.json`) to exercise a different model set (see
[MODELS.md](MODELS.md) for which keys each preset selects).

### Python SDK (compiled core → numpy, no round-trip)

Build the pybind11 extension once, then script engagements directly
([`bindings/README.md`](../bindings/README.md) for the full reference):

```bash
./scripts/build-python.sh
export PYTHONPATH="$PWD/bindings:${PYTHONPATH:-}"
```

```python
import gncsim

# One engagement. A dict config mirrors configs/*.json; missing keys use core defaults.
res = gncsim.run({"scenario": "homing", "model": "3dof", "seed": 1})
print(res["miss_distance"], res["intercept"])         # scalar outcome
r = res["series"]["range"]                             # numpy float64 array
print("min range (m):", r.min())                       # == analytic CPA-adjacent sample

# A dispersed Monte Carlo batch — IC dispersion lives in the C++ core (MonteCarlo.cpp),
# deterministic given the seed; `workers` parallelizes but stays bit-identical to serial.
batch = gncsim.monte_carlo({"scenario": "homing",
                            "monte_carlo": {"num_cases": 1000}}, n=1000, workers=8)
print("P(kill):", batch["p_kill"], "intercepts:", batch["intercepts"])
```

The result dict's `series` keys are the same channel names the web app and CSV use, so a
plot script written against the SDK works on a CLI run's CSV unchanged.

---

## 12. References

- Zarchan, *Tactical and Strategic Missile Guidance*, 7th ed. — PN/APN (ch. 2, 4), optimal &
  ZEM/ZEV guidance (ch. 8), weaving-target case.
- Ben-Asher & Yaesh, *Advances in Missile Guidance Theory* — predictive/optimal ZEM/ZEV.
- Bar-Shalom, *Estimation with Applications to Tracking and Navigation* — α-β (§6), EKF
  (§6, §10), IMM (§11), measurement geometry (§3).
- Bar-Shalom & Fortmann, *Tracking and Data Association* — PDA/JPDA, validation gating.
- Blackman & Popoli, *Design and Analysis of Modern Tracking Systems* — M-of-N lifecycle, MHT.
- Julier & Uhlmann — Covariance Intersection.
- Blom & Bar-Shalom (1988) — the IMM algorithm.
- Skolnik, *Introduction to Radar Systems* — radar measurement & range equation, Swerling.
- Gandhi & Kassam, *IEEE T-AES* 24(4), 1988 — CA-CFAR threshold and Pd/Pfa.
- Richards, *Fundamentals of Radar Signal Processing* — CFAR, detection.
- Hudson, *Infrared System Engineering* — NETD, IR contrast.
- Stevens & Lewis, *Aircraft Control and Simulation* — rigid-body EOM, quaternion kinematics.
- Vallado, *Fundamentals of Astrodynamics and Applications* — ECI/ECEF, J2, WGS-84, geopotential.
- *U.S. Standard Atmosphere, 1976* (NOAA/NASA/USAF).
- IEEE Std 952 — Allan-variance inertial-sensor characterisation.
- Hosein & Athans, *Weapon-target assignment*; Bertsekas, *Auction algorithms*.
- Driels, *Weaponeering* — Carleton/Gaussian lethality; Wagner et al., *Naval Operations
  Analysis* — shoot-look-shoot.
- Sänger–Bredt skip-glide; Allen–Eggers reentry; Wertz, *Orbit & Constellation Design*.
