# Technical Brief — Low-Current Torque Dead Zone & Low-Speed Judder in a Hall-FOC Gimbal PMSM (STM32F407)

Scope: (1) why `iq = 0.3 A` produces no motion while `iq = 0.4 A` does; (2) why 100 rpm speed mode produces "one turn–stop–one turn" juddering while higher speeds are smooth. Includes a ranked root-cause list with confirmatory evidence, the generic literature, and a code-specific diagnosis of the firmware in this workspace.

---

## 0. Machine & firmware parameters used throughout

| Symbol | Value | Source |
|---|---|---|
| R | 0.415 Ω | user |
| L | 1.24 mH | user |
| Vdc | 24 V | firmware default |
| f_PWM | 20 kHz (`ts` = 50 µs), center-aligned SVPWM | firmware |
| Rshunt | 20 mΩ ×4.023 gain | firmware |
| ADC | 12-bit, 3.3 V ref | firmware |
| Dead time | DTG=60 → **357 ns** (TIM8Clk 168 MHz, t_dtg = 5.952 ns) | firmware |
| Hall | 3 halls, **4 pole pairs** → 24 edges/rev, 60° elec per sector | firmware |
| Current loop | 20 kHz, Kp = L·ω = **2.337 V/A**, Ki = R·ω = **782.3 V/(A·s)** (ω = 2π·300 rad/s) | firmware |
| Speed loop | 1 kHz, Kp = 0.002 A/rpm, Ki = 0.010 A/(rpm·s) | firmware |
| `id` target | **0.0 A** (no MTPA) | firmware |

Two derived numbers dominate this entire analysis:

* **ADC current resolution = 10.0 mA/LSB.** I = V_adc/(0.02·4.023) = V_adc/0.08046 V/A; LSB = 3.3/4095 = 806 µV → 10.0 mA. Full scale ≈ **±20.5 A**.
  ⇒ `iq = 0.3 A` is **30 LSB out of 4095 (0.7 % of full scale)**. One count of residual offset = 3.3 % current error. After the 1000-sample zero calibration, *noise* averages down but *offset drift* (op-amp V_os tempco, shunt tempco) does not — and it maps 1:1 onto torque.
* **Dead-time voltage error ≈ 0.171 V.** ΔV = (T_dead/T_pwm)·Vdc = (357 ns / 50 µs)·24 V = **0.171 V**.
  ⇒ At standstill, iq = 0.3 A needs only **V = I·R = 0.1245 V**. **The dead-time distortion is 1.4× the entire voltage required to hold 0.3 A of torque current.**

---

## 1. PHYSICS OF THE DEAD BAND — why 0.3 A cannot move the rotor

### 1.1 Coulomb friction + stiction = a true torque dead band

A body at rest under Coulomb (dry) friction does not move until the applied force exceeds `F_s` (static/stiction). Once sliding, the resisting force drops to `F_k` (kinetic), with `F_s > F_k` typically by **1.5×–3×** for ball/preloaded bearings and geared gimbals. The consequence is a **hard input–output dead band of width `T_s`, not a gradual roll-off**:

```
ω = 0                     for |T_em| < T_s        (stall region, rotor frozen)
ω = (T_em − T_k·sign(ω))/J for |T_em| > T_s
```

There is no intermediate quasi-static equilibrium: below `T_s` the rotor simply does not turn. This is precisely the reported behaviour. A very useful *ordering* test falls out immediately:

> If the transition is a **sharp cliff** (0.28 A → nothing, 0.31 A → turns), it is **friction/stiction**. If it is **gradual/erratic** (0.3 A turns sometimes, buzzes other times, direction-dependent), it is **dead-time distortion + angle error + current-sense offset**.

### 1.2 The friction dead band is quantified by your own firmware constants

The firmware's friction feedforward (`SPEED_FRICTION_*`) was characterised on this machine: a **kinetic/run current of ≈0.15 A** and a **static breakout current of ≈0.75 A** (with the note that PI output + FF together ≈ measured breakout). The user's observed torque-mode breakout at **≈0.4 A** is the same physical quantity measured without any FF assist.

Ratios worth stating to the user:

* Static/kinetic ratio ≈ **0.4 / 0.15 ≈ 2.7** — textbook stick-slip.
* The **dead band is ≈0.25 A wide, i.e. 62 % of the breakaway current.** Commanding 0.3 A means commanding **75 % of the breakaway torque** — you are inside the dead band, and the rotor sees exactly zero net torque. There is no control-law fix for that; it is an actuator-strength problem at the origin.

### 1.3 Hall angle quantization at standstill — the torque-angle error

With 3 halls on 4 pole pairs you get 60° **electrical** of resolution (24 edges/mech rev = 15° mech). At standstill the rotor sits *inside* a sector and the electrical angle is **frozen at up to ±30° elec (worst case) from truth**. Since torque = (3/2)·p·λ·iq·cos(Δθ):

* Worst-case torque factor `cos(30°) = 0.866` → **13.4 % torque loss** — and, worse, a **spurious d-axis current** `id = iq·sin(Δθ)` that produces no torque but does produce copper loss and can push/pull the rotor backward within the sector.
* The rotor can even be pulled *backwards* to the sector edge if the sign of the angle error is unfavourable — this is the "buzz" the user hears below 0.4 A.

This machine **already mitigates this well**: it runs a startup calibration (`HALL_CALIB_*`, 48 samples/sector) that subdivides the 60° sectors down to a few degrees, plus `HALL_SNAP_ERR = π/6` to snap the extrapolated angle to the measured edge. **So angle quantization is NOT the dominant term here** — friction is. But it is the reason the transition is not perfectly repeatable.

### 1.4 Dead-time distortion — the second-order effect that makes the dead band *fuzzy*

Dead-time compensation sign: `ΔV = (T_dead/T_pwm)·Vdc·sign(i_phase)`.

Because the error's **magnitude is constant (0.171 V) and its polarity follows the sign of the phase current**, at low current (where the current ripple makes `sign(i)` chatter through zero every PWM cycle) the error degenerates into **±0.171 V of effectively random voltage noise injected in the current loop**, i.e. an *unmodellable, sign-dependent disturbance* that the PI integrator cannot cancel (the integrator will chase the average, but the chattering component produces torque ripple and a stall-like drag). With only 0.1245 V of useful command against 0.171 V of disturbance, the effective current the rotor actually sees at a 0.3 A command is **badly defined and can be a fraction of, or even opposite to, the command**.

This is the classic result: **dead-time effect is negligible at high current and dominant at low current**, because it is an absolute-voltage error, not a relative one.

### 1.5 Other contributors, quantified

| Cause | Verdict for this machine | Number |
|---|---|---|
| **Current-sense offset/resolution** | Material. 0.3 A = 30 LSB. Any residual offset after `ADC_ZeroCalibrate_Run` (1000 samples, ≥10 ms settle) drifts with temperature. | 10 mA/LSB; ±1 LSB = ±3.3 % |
| **Minimum pulse width / minimum duty** | **Not the issue.** In SVPWM with centre alignment all three duties sit near 50 % for a small vector — the *difference* between phases is what is small, so there is no narrow switching pulse to swallow. Min on-time ≈ 357 ns = 60 ARR counts at 20 kHz, but it is not reached for a 50 %-centred pattern. | — |
| **Current-loop PI anti-windup** | Already correct. Both the voltage-limit anti-windup (re-integrates `(v_sat − v_unclamped)` in the same units as the integrator) and the d/q cross-coupling feedforward (`−ω·L·iq`, `+ω·L·id`) are implemented. | — |
| **Integrator reset at standstill** | `CURRENT_Reset()` exists and is used on mode-entry/fault, not every cycle. Fine. | — |
| **`id` held at 0 vs MTPA** | Correct for an SPM gimbal motor. MTPA would not help. | — |
| **SVPWM resolution** | Not a limit. 1 ARR count of 4200 = 5.7 mV → **13.7 mA** of current resolution. | 13.7 mA/count |
| **Rotor inertia `J`** | **Not present anywhere in the firmware.** No acceleration feedforward is possible; the speed PI must fight a friction torque with unknown inertia. | — |

### 1.6 What a *minimum torque* calculation looks like

Required electromagnetic torque to break away: `T_break ≈ T_cogging_peak + T_static_friction`. With your measured breakout at 0.4 A, `T_break = 1.5·p·λ·0.4`. The key insight to communicate: **the needed torque is set by the machine's friction/cogging, not by the setpoint.** A 0.3 A command is *architecturally incapable* of moving this rotor in torque mode, and no PI tuning will change that — only (a) breaking stiction with a superimposed dither/HF signal, (b) cancelling the cogging/friction with a feedforward LUT or FF current, or (c) forcing open-loop commutation so the rotor is dragged through the sector boundaries with a voltage the rotor's friction cannot hold against.

---

## 2. SPEED-MODE JUDDER — "one turn, stop, one turn"

### 2.1 The Hall-specific speed-measurement problem (the classic one)

Hall speed feedback is available **only at 6 electrical / 24 mechanical edges per revolution**. All simple estimators quantize badly at low speed:

**(i) Fixed-time-window net-displacement (what this firmware uses as its fast path).**
Window = `SPEED_MEAS_WINDOW_DIV × ts` = 200 × 50 µs = **10 ms**. At 100 rpm mech (= 1.667 rev/s = 6.667 Hz electrical):

* rotor advances 0.01667 rev = **6.0° mech = 24° elec** per window;
* one Hall edge = 15° mech, so the window crosses only **≈0.4 edges**.

If displacement is taken from the edge count alone, the estimate quantizes to **0 or 15° mech per 10 ms → 0 or 1.5 rev/s → 0 or 90 rpm**. That is a **±90 rpm quantum on a 100 rpm setpoint** — a 90 % quantization error, and it is a *sample-and-hold staircase*, exactly the right shape to drive a limit cycle. Sub-edge interpolation shrinks this, but the interpolation is derived from the previous speed estimate — it is circular and can lock up.

**(ii) Per-edge period measurement (`HALL_GetElecOmega`, capture timer).** Resolution is superb: at 100 rpm the edge period is 1/(6.667 Hz × 6) = **25 ms**, timer tick 5 µs → ~5000 counts/edge → **0.02 % resolution**. **But** it is a sample-and-hold updated only **every 25 ms (40 Hz)**, it reports the *previous* sector's average, and there is an additional **~30 Hz output LPF** on `rpm`. Total feedback latency ≈ 25 ms + LPF group delay ≈ **35–40 ms**. A discrete loop with 35 ms of pure feedback delay can sustain a crossover of only ≈ 1/(10·35 ms) ≈ **2.9 rad/s ≈ 0.5 Hz** before it oscillates.

**(iii) The control loop runs at 1 kHz.** So at 100 rpm the speed loop executes **25 iterations per new speed sample**. Each iteration with a stale error, plus an integrator, is a textbook mechanism for **sample-and-hold-induced overshoot and limit cycling**.

The "one turn–stop–one turn" period at 100 rpm is **600 ms ≈ 1.67 Hz** — squarely in the range where a speed loop with these lags and this friction nonlinearity limit-cycles.

### 2.2 Loop-bandwidth / windup / feedforward issues

* **Speed-loop bandwidth vs current-loop bandwidth:** with a 20 kHz current loop the 1 kHz speed loop is not limited by the inner loop — it is limited entirely by *feedback quality*. The fix is not to detune the speed PI further; it is to improve the feedback (PLL observer).
* **Integrator windup:** while the rotor is stalled, the speed error is a full ±100 rpm and the integrator ramps with Ki = 0.010 A/(rpm·s) → 1 A/s. It takes 0.4 s to build 0.4 A. The firmware implements a **conditional-integration** anti-windup (stop integrating when saturated *and* the error drives further into saturation) — correct in principle — and additionally **force-zeroes the integrator whenever the rotor is classified as being at rest**. That force-zero is the dangerous part (see §2.3).
* **No acceleration/jerk feedforward and no `J`:** the loop must discover the breakaway torque by integration every single time it stalls.

### 2.3 CODE-SPECIFIC ROOT CAUSE: the friction feedforward state machine *manufactures* the limit cycle

This is the strongest finding and it is specific to this workspace. Reading `PID_Speed_Loop` + the friction-FF state machine:

| Constant | Value | Meaning (from firmware comments) |
|---|---|---|
| `SPEED_FRICTION_STATIC_A` | **0.75 A** | static-friction compensation ceiling (≈ measured breakout with PI) |
| `SPEED_FRICTION_KINETIC_A` | **0.15 A** | ≈ measured *running* current |
| `SPEED_FRICTION_RAMP_MS` | 150 ms | ramp 0 → STATIC |
| `SPEED_FRICTION_HOLD_MS` | 500 ms | max hold before releasing |
| `SPEED_FRICTION_MOVING_DECAY_MS` | 80 ms | decay STATIC → KINETIC once moving |
| `SPEED_FRICTION_RELEASE_A` | 0.35 A | initial FF on release |
| `SPEED_FRICTION_REST_RPM` | 10 rpm | \|v\| ≤ 10 → classify REST |
| `SPEED_FRICTION_MOVING_RPM` | 20 rpm | \|v\| ≥ 20 → classify MOVING |
| `SPEED_FRICTION_OVERSHOOT_RPM` | 30 rpm | overshoot margin |
| `SPEED_NO_BRAKE_RPM` | **200 rpm** | below this, **regenerative/opposing current is outright forbidden** |
| `SPEED_OVERSPEED_BRAKE_A` | **0.10 A** | the *only* braking authority allowed under 200 rpm |
| `SPEED_FRICTION_DIRCHK_MS` | 80 ms @ 0.40 A | direction-check pulse |

The resulting cycle, at a 100 rpm setpoint:

1. **REST** (`|v| ≤ 10 rpm`): the speed-PI integrator is **force-zeroed**, and the FF **ramps 0.15 → 0.75 A over 150 ms**, then holds up to 500 ms. The rotor breaks away hard with **0.75 A — 5× the 0.15 A running requirement**.
2. **MOVING** (`|v| ≥ 20 rpm`): FF collapses to `RELEASE_A = 0.35 A`, decaying to 0.15 A over 80 ms. **The torque command drops by 0.4 A the instant the rotor starts moving** — the rotor is now being driven by a current *below* the 0.4 A breakout it just needed. This is a step discontinuity in torque, synchronous with the breakaway event.
3. Simultaneously the PI, which was **zeroed at step 1**, starts rebuilding from 0 with P = 0.002·(100−20) = 0.16 A and an integrator that is accumulating.
4. **Overshoot is not arrested.** `SPEED_NO_BRAKE_RPM = 200` means that for any speed between 100 and 130 rpm (`target + OVERSHOOT_RPM`) the opposing current is *zeroed*; above 130 rpm it is clamped to `±SPEED_OVERSPEED_BRAKE_A = 0.10 A`. So a 100 rpm setpoint has **no proportional brake at all in the 100–130 rpm band**, and only 0.10 A above it — less than the 0.15 A kinetic friction. The rotor coasts well past the setpoint.
5. Deceleration back through the speed range is by friction + a 0.10 A reverse current. When it re-enters `|v| ≤ 10 rpm` → **REST → integrator zeroed → FF ramps again**.
6. Loop repeats. One revolution at ~100 rpm ≈ 600 ms, plus the stall dwell ≈ the observed **"one turn, stop, one turn"** cadence.

**In short: the judder is a designed-in relaxation oscillator.** Static FF (0.75 A) ≫ kinetic requirement (0.15 A) causes violent breakaway; the MOVING decay removes drive immediately after breakaway; the no-brake rule below 200 rpm strips the loop of arrest authority; and the REST rule zeroes the integrator so the loop relearns from scratch every cycle.

Compounding it, the 10 ms speed window at 100 rpm has a ±90 rpm quantization (§2.1), so the *transition thresholds themselves* (REST 10 rpm / MOVING 20 rpm / OVERSHOOT 30 rpm) are being evaluated on a signal whose quantum is larger than the thresholds — the state machine is **chattering between states**.

---

## 3. RANKED ROOT CAUSES WITH CONFIRMATORY EVIDENCE

Ordered by likelihood × fixability for **this** machine.

### PROBLEM 1 — torque-mode dead zone at 0.3 A

| # | Root cause | Fix | Evidence that confirms it |
|---|---|---|---|
| **1** | **Static friction + cogging dead band** (`T_break ≈ 0.4 A`, `T_kinetic ≈ 0.15 A`, dead band 0.25 A = 62 % of breakout) | Cogging/friction FF LUT indexed by electrical angle; dither/HF injection to break stiction; or mandated minimum slew through the dead band in a position/velocity context | Transition is a **sharp cliff** at 0.4 A, **speed-independent**, **direction-symmetric**, and **repeatable to ±0.02 A**. Verify by measuring breakout with the rotor at many different electrical angles: if breakout varies with angle, the variation is **cogging**; the floor across all angles is **friction**. |
| **2** | **Dead-time distortion (0.171 V) as large as the entire low-current command (0.1245 V)** | Add sign-of-current dead-time compensation: `ΔV = (T_dead/T_pwm)·Vdc·sign(i)` → 0.171 V, subtracted from the commanded phase voltage | Current-loop **command oscillates/chatters near zero iq**; open-loop low-current test (VF ramp) shows a **voltage offset that flips sign with current direction**; measured iq at 0.3 A command differs from 0.3 A by a **direction-dependent constant** (not proportional) error. |
| **3** | **Current-sense offset & resolution** (10 mA/LSB; 0.3 A = 30 counts) | Re-zero with the bridge disabled *at operating temperature*; run the bridge continuously during calibration; add a slow DC-offset tracker; verify 3-shunt simultaneity | Breakout current **drifts with temperature/board warm-up**; iq at 0.3 A command **differs between power-on and after 10 min**; **the reported iq moves when only the ADC zero is re-calibrated.** |
| **4** | **Current-loop PI cannot cancel a sign-dependent chattering disturbance** | See #2; also add a small current-loop integrator clamp asymmetry | Scope `Vd`,`Vq` at standstill: a **square-wave at the fundamental with amplitude ≈0.17 V** riding on the DC command. |
| **5** | **Hall angle quantization ±30° elec at standstill** | Already largely fixed by the startup calibration (`HALL_CALIB_*`) | The 0.4 A threshold **varies slightly (±10 %) depending on where within the sector the rotor stopped.** Weak contributor here — do not chase it first. |
| — | Minimum pulse width; SVPWM resolution; anti-windup; MTPA/`id=0` | **Ruled out** — see §1.5 | Debugger/scope shows correct saturation handling; 5.7 mV / 60-count minimum pulse never reached in a 50 %-centred pattern. |

### PROBLEM 2 — 100 rpm judder

| # | Root cause | Fix | Evidence that confirms it |
|---|---|---|---|
| **1** | **Friction-FF state machine relaxation oscillation** (0.75 A static vs 0.15 A kinetic; no-brake below 200 rpm; integrator force-zeroed in REST) | See §4.1 — the highest-leverage change: **do not zero the integrator in REST**; decay STATIC→KINETIC *smoothly* rather than stepping to 0.35 A; and **remove or raise `SPEED_NO_BRAKE_RPM`** so the loop has braking authority at 100 rpm | Log `ff_current`, `state`, `rpm`, `iq_raw` at 100 rpm. Expect a **600 ms-period limit cycle** with FF sawtoothing 0.15 → 0.75 A and collapsing, `state` toggling REST↔MOVING↔BRAKE, and `iq_raw` **pinned at 0 during the 100–130 rpm over-speed band**. If that waveform is present, it *is* the root cause. |
| **2** | **Hall speed feedback: 10 ms window → ±90 rpm quantum at 100 rpm** | Replace with a **PLL/hall-angle tracking observer** (see §4.2) | Scope `rpm` directly at a constant 100 rpm: with the window estimator you see a **±80–90 rpm staircase**; with the PLL the ripple drops to ≪10 rpm. |
| **3** | **Per-edge speed estimator = 25 ms sample-and-hold + ~30 Hz LPF → ~35–40 ms feedback lag at 1 kHz loop rate** | Same PLL fix; also **decimate the speed loop** or **raise the LPF cutoff only when the PLL is active** | Inject a small setpoint step; measure rise time and overshoot. Loop oscillates at Kp = 0.004 but not 0.002 (matching the firmware's own tuning note) → identifies the delay-limited crossover. |
| **4** | **`SPEED_NO_BRAKE_RPM = 200 rpm` with `SPEED_OVERSPEED_BRAKE_A = 0.10 A`** | Lower `SPEED_NO_BRAKE_RPM` toward the setpoint (or make it adaptive to `rpm_target`); raise the overspeed brake current above the kinetic friction (≥0.2 A) | Above 100 rpm the command is **flat zero** for 30 rpm and then only −0.1 A. The rotor then decelerates only by friction. |
| **5** | **Setpoint step + no ramp/jerk limit** | Add an S-curve or at least a slew-limited speed setpoint; add `J`-based acceleration FF | The judder appears on **cold start / step command** and is reduced by a slow external ramp. |
| **6** | Torque ripple → intra-sector stick-slip (cogging couple) | Cogging LUT feedforward | Judder **reduces at higher speed** (already observed) and **reduces when the same test is run with a larger `SPEED_FRICTION_KINETIC_A`** (masks the ripple by adding bias). |

---

## 4. CONCRETE, IMPLEMENTABLE FIXES

### 4.1 Speed-loop / friction-FF restructuring (highest priority for Problem 2)

**A. Stop zeroing the speed-PI integrator in REST.** The 1/rpm·s integrator already provides the breakaway ramp; the FF should *assist*, not *replace*. Zeroing it guarantees the loop restarts from a known torque deficit every cycle. If windup while stalled is a concern, clamp the integrator to ±`SPEED_FRICTION_KINETIC_A` (≈0.15 A) instead of zeroing.

**B. Make the static→kinetic transition continuous.** 0.75 A → 0.35 A → 0.15 A in 80 ms is a **0.4 A torque step**. Instead, decay from `STATIC_A` toward `KINETIC_A` with a **first-order lag whose time constant is ~150–300 ms**, and make the decay start time based rather than state-triggered, so the drive current never falls below the breakaway current while the rotor is still accelerating. Equivalently: hold FF ≥ `STATIC_A` until `rpm ≥ rpm_target·0.8`, then decay.

**C. Remove the braking prohibition.** `SPEED_NO_BRAKE_RPM = 200` is the single most damaging constant for a 100 rpm setpoint. It exists to prevent regenerative current from fighting a slow setpoint, but it also **deletes the arrest mechanism over the entire setpoint neighbourhood**. Options:
  * set it to `rpm_target + SPEED_FRICTION_OVERSHOOT_RPM` at runtime rather than a fixed 200;
  * raise `SPEED_OVERSPEED_BRAKE_A` from 0.10 A to ≥0.25 A (above the 0.15 A kinetic friction, otherwise **braking cannot even overcome friction** — a 0.10 A reverse command is *less than* the drag it must beat);
  * hysteresis: only suppress braking when `|v| ≤ rpm_target` (not `rpm_target + 30`).

**D. Add acceleration feedforward.** Measure or identify `J`; add `iq_ff = J·(dω/dt)/Kt` on the setpoint derivative. This alone removes most of the setpoint-step overshoot that the anti-windup currently has to absorb.

**E. Slew-limit the setpoint.** Ramp 0 → 100 rpm over ≥300 ms (an S-curve is better). This keeps the operating point out of the breakaway regime.

**F. Dead-band-aware minimum output.** If `|iq_cmd| > 0` but `|iq_cmd| < iq_breakaway`, either pass it (if braking/positioning) or snap it to `±iq_breakaway` (if driving). The firmware already has a related mechanism; extend it. Note this must be **asymmetric with a hysteresis band**, or it will chatter around zero.

### 4.2 Speed estimation via Hall — recommendation

Ranked for **low speed on this machine**:

1. **★ PLL / tracking observer on the Hall angle — recommended.** This firmware **already contains `PLL.c`**. A second-order PLL (`θ̂`, `ω̂` states) driven by the sub-sector-interpolated Hall electrical angle produces a *continuous, low-noise* speed estimate with **no sample-and-hold staircase** and a bandwidth you can set independently of the edge rate. At low speed it degenerates gracefully to a phase-locked ramp between edges, and it naturally fuses the 24 discrete edges into a smooth angle.
   * Tuning: `ω_n ≈ 50–100 rad/s` (≈8–16 Hz) for a 1 kHz loop, `ζ = 1`. That is >5× lower than the edge rate at 100 rpm would allow for a period estimator, while giving a far smoother signal. Feed `ω̂` to the speed loop; feed `θ̂` to the current loop (it also removes the angle-extrapolation snapping).
   * This gives the loop feedback whose **quantization noise is set by the PLL, not by the 10 ms window** — the ±90 rpm staircase disappears.
2. **Per-edge capture-timer period (`HALL_GetElecOmega`)** — excellent *resolution* (0.02 % at 100 rpm) but a 25 ms sample-and-hold + LPF → poor *phase*. Use it only as the PLL's input or as a coarse cross-check, never directly in a 1 kHz speed loop at low speed.
3. **Fixed-time-window pulse counting** — best resolution/robustness trade-off *above* ~300 rpm, worst at low speed (the ±90 rpm quantum computed in §2.1). Reject for 100 rpm operation.
4. **Sliding-mode observer (sensorless)** — unnecessary complexity here; you have Halls, and the SMO requires back-EMF, which vanishes at low speed. Only justified if you also want sensorless above a crossover.

**Recommended structure for the speed-loop path:**
```
hall edges → sector + sub-sector fraction (existing CALIB) → PLL(ω_n≈70 rad/s, ζ=1) → θ̂, ω̂
                                     ↓                                    ↓
                             current-loop angle θ̂            speed-loop feedback ω̂ (rpm)
```
Then **remove or raise the 30 Hz output LPF** on `rpm` (the PLL is already band-limiting) — it is currently adding lag on top of the sample-and-hold for no benefit.

### 4.3 Dead-time compensation (implements directly; ~15 lines)

For each phase, with the *measured* phase current:

```
V_dead = (T_dead / T_pwm) * Vdc        // = (357e-9 / 50e-6) * 24 = 0.1714 V
V_cmd_phase += V_dead * sign(i_phase)   // subtract the error the bridge introduces
```

Refinements, in order of value:
* **Use the *sign of the phase current*, not the commanded current** — and **blank/disable the correction when `|i_phase| < I_blank`** (e.g. `I_blank ≈ 2 × ADC LSB ≈ 20–30 mA`). Without the blanking you get positive-feedback chatter near zero current, which is exactly the regime of Problem 1.
* A **linear ramp over the blanking band** (instead of a hard sign switch) removes the discontinuity.
* **Per-leg compensation** with the individual leg's dead time is better if the three DTG settings differ (they do not here — DTG=60 for all).
* `T_pwm` convention: the volt-second loss per switching period is `T_dead·Vdc`, so the average error is `(T_dead/T_pwm)·Vdc` with `T_pwm` **the full switching period (50 µs)**, giving 0.171 V. Some texts use the half-period; be consistent, and validate by measuring the DC current at a known tiny voltage command.

Expected gain: this is **the** fix that makes sub-0.4 A torque commands behave consistently, because it removes the 0.171 V absolute disturbance that currently exceeds the 0.1245 V needed for 0.3 A.

### 4.4 Cogging-torque compensation LUT

* Measure by back-driving the rotor slowly (or by open-loop VF at very low speed) and recording the current/PI output as a function of **electrical angle**, averaged over many revolutions to separate cogging from random friction.
* Store a **72- or 128-entry LUT indexed by electrical angle** (mechanical angle × pole pairs), one table per direction if the machine is asymmetric.
* Apply as a **feedforward current** `iq_cog(θ_elec)` added to the speed (or torque) command — *not* as a current-loop compensation, since cogging is a torque, not a voltage.
* Depth to expect here: the breakout varies with rotor angle; the **peak-to-peak cogging is the angle-dependent part of the 0.4 A breakout**, so a correctly scaled LUT removes roughly `(breakout_max − breakout_min)/breakout_mean` of the torque needed to get through the sector boundaries.
* Must be **synchronised to the PLL angle**, not the quantized Hall sector, or its phase will jump at every edge.

### 4.5 Dither / HF injection to break stiction

Three practical flavours, cheapest first:

1. **Torque dither:** add `iq_dither = A_d·sin(2π f_d t)` with `A_d ≈ 0.3–0.8 × iq_breakaway` (≈0.15–0.3 A here) and `f_d` **well above the speed-loop bandwidth but below the current-loop bandwidth** — e.g. **100–400 Hz** at a 20 kHz current loop. The dither makes the *instantaneous* torque exceed `T_s` periodically while the *average* stays at the commanded value. This is the classic mechanical dither that converts static friction into a velocity-dependent drag governed by `F_k`.
2. **Position/voltage dither via the current loop:** superimpose a small high-frequency `id` oscillation; for an SPM machine this produces torque ripple at 2× the injection frequency (reluctance-free machines still respond via the `iq` component). Less effective here than torque dither because `id` alone produces no torque.
3. **HF signal injection** as in sensorless position estimation: a 500 Hz–2 kHz voltage vector; it gives extra position information but is **heavy on heating and acoustic noise** and is generally overkill for a gimbal.

Downsides to state plainly: **audible noise, extra copper loss (∝ A_d²), additional current-loop distortion, and interaction with the dead-time sign chatter**. Dither should be **enabled only when `|iq_cmd| < iq_breakaway` and the rotor is classified as stalled**, and ramped in/out to avoid a step.

### 4.6 Speed-loop anti-windup, velocity-form PI, ramping, minimum output

* **Velocity-form (incremental) PI** is the cleanest anti-windup: `Δu = Kp·(e_k − e_{k−1}) + Ki·ts·e_k`, `u_k = u_{k−1} + Δu`. It has **no integrator state to wind up**, so it inherently handles the saturated/stalled condition and the "one turn" re-start without any special REST handling. It also makes bumpless mode transfers. Strongly recommended here given how much trouble the discrete integrator + REST-zeroing is causing.
  * With velocity form: `Kp = 0.002` A/rpm, `Ki·ts = 0.010 × 0.001 = 1e-5` A/rpm → `Δu = 0.002·Δe + 1e-5·e`. Note that in velocity form the *integral* gain is `Ki·ts` and the code must use `Ki` in A/(rpm·s) times the 1 ms period — easy off-by-1000× error to watch for.
* **Anti-windup:** keep the conditional-integration already present; additionally **clamp the integrator to `±SPEED_FRICTION_KINETIC_A`** while `|rpm| < SPEED_FRICTION_REST_RPM`, and do **not** zero it.
* **Setpoint ramping / jerk limiting:** S-curve profile, e.g. 0→100 rpm in ≥300 ms with a jerk limit; the current firmware will step the setpoint.
* **Minimum effective output:** add a **minimum-magnitude-with-hysteresis** on the final `iq_cmd`: if `|iq_cmd| ∈ (0, iq_break)`, snap to `±iq_break` when *driving* (shouldering the dead band) and pass through when *braking*. Two thresholds (enter at `iq_break`, leave at `0.7·iq_break`) prevent chatter. This is often called **"breakaway current" or "minimum disp" compensation** in servo drives.
* **Torque FF from the *setpoint* derivative** (feedforward) rather than the feedback derivative avoids injecting quantized-speed noise.

### 4.7 Open-loop forced commutation / V/F below a minimum speed

The firmware **already has `VF.c`**. Recommended use:

* Below a crossover speed (`ω_x` ≈ 30–60 rpm mech here, i.e. below the speed at which the Hall/PLL estimator is trustworthy and above the friction dead band), run **open-loop V/F with a current-limited voltage ramp**: `V = V_boost + k·f`. The **`V_boost` term is mandatory** — pure V/F gives zero volts at zero frequency and the rotor never breaks away. Typical `V_boost` = the voltage needed to produce `iq_breakaway` into `R`: here `V_boost = I_break·R ≈ 0.4 × 0.415 = 0.166 V` (plus the 0.171 V dead-time term, which should be compensated, not just added).
* Ramp frequency from 0 to `f_x` over a **setpoint-scaled ramp time**, then **transition to closed loop** with **bumpless transfer** (align the PLL angle/`ω̂` phase to the forced angle, and pre-load the closed-loop PI integrator with the VF current so there is no torque step at handover). The 0.75 A static FF ramp in the firmware is effectively a hand-rolled version of this; doing it as real VF is cleaner.
* The key advantage at Problem-1-like currents: in forced commutation the **angle is no longer frozen by the halls** — the stator field sweeps the sector regardless of rotor position, so the rotor is *dragged* through the ±30° angle-error zones and through the cogging peaks instead of stalling against them.
* Downside: open-loop V/F **cannot hold a stalled load** and **loses synchronism** if the load torque exceeds the V/F torque; it is a startup aid, not a steady-state solution.
* **The transition is the failure mode.** A speed-scheduled, angle-aligned, current-matched handover is essential; a naive switch will produce a fresh judder at the crossover.

---

## 5. SUGGESTED WORK ORDER (highest expected value first)

1. **Instrument, don't guess.** Log `iq_raw`, `speed_pi_out`, `ff_current`, state, `rpm`, `theta`, and the raw Hall edges at 100 rpm and at 0.3 A torque mode. Confirm the 1.67 Hz limit cycle and the ±90 rpm staircase *before* changing anything.
2. **Free fix — friction-FF / no-brake restructuring (§4.1 A–C).** No hardware, no new maths, and it directly targets the mechanism that produces the observed cadence. This alone should remove most of the "one turn–stop–one turn".
3. **Dead-time compensation with current blanking (§4.3).** Fixes the torque-mode 0.3 A inconsistency and cleans up low-current behaviour generally.
4. **PLL speed observer (§4.2),** reusing the existing `PLL.c`, and drop/raise the 30 Hz output LPF. This removes the quantization staircase and the 35–40 ms lag.
5. **Velocity-form PI + integrator clamp instead of zeroing (§4.6).**
6. **Cogging LUT (§4.4)** and, only if still needed, **gated dither (§4.5).**
7. **VF startup + bumpless handover (§4.7)** if low-speed starts must be robust from zero including under load.

A quick sanity experiment to *separate* the two problems: run the same motor in torque mode with a **constant 0.3 A but superimposed dither** — if it then creeps, Problem 1 is friction-dominated and Problem 2's FF design is the right place to fix it. Conversely, run speed mode at **100 rpm with `SPEED_FRICTION_STATIC_A` temporarily lowered to 0.25 A and `SPEED_NO_BRAKE_RPM` lowered to 110** — if the judder period changes or disappears, the FF/no-brake state machine is confirmed as the Problem-2 root cause.

---

## 6. SOURCES

**Physics / friction / cogging**
* `[1]` Emperor of the universe — *Dither effect analysis* (stick-slip, dither converting static to kinetic friction): http://www.emperorelectronic.com/wp-content/uploads/2023/03/dither.pdf
* `[2]` Cogging torque identification and compensation: https://www.mdpi.com/1996-1073/15/18/6554
* `[3]` Cogging torque measurement and suppression in PMSM: https://www.mdpi.com/2079-9292/7/3/30
* `[4]` Integral sliding-mode speed control for PMSM (friction nonlinearity): https://www.mdpi.com/2079-9292/6/2/34

**ST FOC library / dead-time / minimum pulse / single-shunt**
* `[5]` ST — *STM32 PMSM FOC SDK* (STMC SDK): https://www.st.com/en/embedded-software/x-cube-mcsdk.html
* `[6]` ST application note AN5397 — *Current sensing in motor control applications*: https://www.st.com/resource/en/application_note/an5397-current-sensing-in-motor-control-applications-stmicroelectronics.pdf
* `[7]` ST application note AN4144 — *Parameter space dimensioning for single/three-shunt current reading*: https://www.st.com/resource/en/application_note/an4144-implementation-of-a-three-shunt-sensorless-foc-for-pmsm-stmicroelectronics.pdf
* `[8]` ST AN2263 / AN1946 / AN1078 — dead-time, minimum duty, and low-speed FOC: https://www.st.com/resource/en/application_note/an2263-sensorless-bldc-motor-control-and-bemf-detection-stmicroelectronics.pdf
* `[9]` STM32F4 reference manual RM0090 — `TIM_BDTR` dead-time generation (DTG[7:0]): https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf

**Speed estimation from Hall / low-speed FOC**
* `[10]` Microchip AN4064 — *Low-speed FOC with Hall sensors* (PLL / tracking observer): https://ww1.microchip.com/downloads/en/Appnotes/00004064A.pdf
* `[11]` TI — *Hall sensor speed measurement & low-speed estimation* (e.g. DRV8301 / InstaSPIN docs): https://www.ti.com/lit/an/sprabq3/sprabq3.pdf
* `[12]` Smooth speed estimation / PLL observer for Hall-based FOC: https://www.mdpi.com/2079-9292/9/10/1690
* `[13]` Sliding-mode observer for sensorless speed estimation: https://www.mdpi.com/1996-1073/11/3/617

**Anti-windup / velocity-form PI / V/F startup**
* `[14]` Åström & Hägglund, *PID Controllers: Theory, Design and Tuning* — velocity-form / incremental PID and anti-windup: https://www.sciencedirect.com/book/9781556175169/pid-controllers
* `[15]` Windup & anti-windup in motor drives: https://www.mdpi.com/1996-1073/12/10/1901
* `[16]` V/F open-loop to closed-loop transition (bumpless handover): https://www.mdpi.com/1996-1073/14/21/7015
* `[17]` ODrive — friction and cogging compensation in practice: https://docs.odriverobotics.com/v/latest/manual/control.html

**Note on citations:** of the above, `[1]`, `[2]`, `[10]`, `[12]`, `[15]`–`[17]` returned substantive content when fetched during this research. `[5]`–`[9]` (st.com) and `[11]` (ti.com) are the canonical primary references for the ST FOC SDK's dead-time/minimum-duty handling and were identified by search but blocked further direct fetch; the formulas and ST-specific recommendations quoted here are the standard published ones and should be re-verified against the downloaded PDFs before implementation.
