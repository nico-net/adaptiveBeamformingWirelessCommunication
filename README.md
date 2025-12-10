# adaptiveBeamformingWirelessCommunication
note: the written files are the correct final versions of the work in each level.

## D0: 2D, 1 BS, LOS (FRIIS), Narrow, DoA (20/11/25-22/11-25)
```LEVEL_D0_final.m```

<details>

<summary> Key Features of this Code: </summary>
Scenario: 2D environment with one Base Station (BS) and two moving User Equipments (UEs).

Channel Model: Line-of-Sight (LOS) using Friis free-space path loss.

Signal Type: Narrowband (single frequency sinusoids).

DoA Estimation: Known from geometry (no algorithmic estimation like MUSIC).

Beamforming: Minimum Variance Distortionless Response (MVDR) adaptive beamforming.

UE Management: Extended UE struct with active/inactive states and DOA tracking.

SINR Calculation: Computes SINR for each UE considering interference and noise.

Visualization: Real-time updating plots for scenario, SINR, and beam patterns.

Replay Functionality: Time slider to review simulation frames.

### Core Workflow:
Initialize Parameters: System (frequency, sampling), geometry (BS/UE positions), and waveforms.

Physics Step: Move UEs each frame based on velocities.

Channel Propagation: Compute received signals at BS using Friis model.

DoA from Geometry: Calculate angles directly from UE-BS positions.

MDL Estimation: Estimate number of active UEs per frame using Minimum Description Length.

MVDR Beamforming: Compute beamforming weights to maximize SINR.

SINR Calculation: Compute SINR for each UE.

Visualization Update: Update plots in real-time.

Data Storage: Store all data for later replay with slider.

### Technical Details:
Array: 8-element ULA with λ/2 spacing.

Noise: AWGN with thermal noise calculation.

DOA: Known exactly from geometry (no estimation error).

Beamforming: MVDR with inverse covariance matrix.

SINR: Calculated considering both interference and noise.

### Novel Aspects:
UE struct with state management (active, tracking, lost).

Per-frame MDL estimation for number of sources.

Real-time visualization with comprehensive plots.

Post-simulation replay with interactive slider.

Combined beam pattern visualization.

</details>

## D1: 2D, 1 BS, LOS (FRIIS), Narrow, LMS (22/11/25-24/11-25)
```LEVEL_D1_plot.m``` 

<details>

<summary> Key Features of this Code: </summary>

Same 2D scenario as D0 but with LMS (Least Mean Squares) adaptive beamforming instead of MVDR

DOA known from geometry, no algorithmic estimation

Key innovation: LMS algorithm with adaptive step size based on input signal power

New analysis: Tracking time since last update vs SINR degradation

New visualization: Received power vs time plot

### Core Workflow:
Initialization: Similar to D0 with parameters, geometry, and UE struct

Physics: UEs move linearly with different speeds

Channel: LOS Friis model

MDL Estimation: Determine number of active UEs per frame

LMS Beamforming:

Updates only when DOA variation exceeds threshold (4°)

Adaptive step size: μ = trace(x*x')

Reference signals: waveform1 for UE1, waveform2 for UE2

SINR Calculation: Computes SINR considering interference and noise

Performance Tracking: Stores "time since last LMS update"

Visualization: Real-time plots including new power vs time

### Technical Details:
LMS Algorithm:

Updates weights: w(n+1) = w(n) + μ·x(n)·e*(n)

Error: e(n) = d(n) - y(n) where d(n) is desired signal

Adaptive μ based on input power for stability

Update Logic: LMS updates only when DOA change > 4°, tracking when performance degrades

Signals: Complex exponentials at different frequencies (fc and fc+1kHz)

Array: 8-element ULA with λ/2 spacing

### Novel Aspects Compared to D0:
LMS vs MVDR: LMS is simpler, sample-by-sample adaptive vs MVDR's matrix inversion

Conditional Updates: LMS only runs when DOA changes significantly, reducing computation

Time Since Update: New metric tracking how long since last beamformer update

Power Tracking: Additional plot showing received power vs time

Adaptive Step Size: μ adapts to input signal power for better convergence

### Key Analysis:
Shows trade-off between update frequency and SINR maintenance

Demonstrates how beamformer performance degrades when DOA changes but weights aren't updated

Illustrates LMS convergence when updates occur

Comparison point: D0 (MVDR) recalculates every frame, D1 (LMS) updates conditionally

### Visualization Updates:
New Subplot (8): Received power vs time for both UEs

SINR Plot: Shows drops corresponding to time since last update

Beam Patterns: Similar to D0 but with LMS-generated weights

Slider: Same replay functionality as D0


</details>

## C0: 3D, 1 BS, LOS (FRIIS), Narrow, LMS (23/11/25-24/11-25)
```C0.m``` or ```C0_1_3D_Pattern.m```
## C: 3D, 1 BS, LOS (FRIIS), OFDM, LMS (wideband) (24/11/25-30/11/25)
```C_ ``` DA SCEGLIERE
## B: 2D, 1 BS, Multipath simplified, OFDM, LMS (wideband) (01/12/25-10/12/25)
```B_noMUMIMO.m``` and ```B_noMUMIMO_channel.m```
## A: 3D, 2 BS, 3GPP, OFDM, LMS (wideband) (11/12/25-)

5 cases of study:
### 1) Simple tracking case in different 3GPP channels
```A_simple.m```

Simple UE around BS to compare different channels scenarios.

### 2) Jammer and null beamforming
```A_jammer.m```

BS is doing beamforming on a UE1 which is transmitting towards it. but another UE2 is jamming. LMS algorithm in bs must be capable of doing null beamforming towards the jammer.

### 3) Train
```A_doppler.m```

This experiment is done in order to see the doppler effect on a real channel. We have a very fast UE (like moving on a train) and the BS is trying to track it. We want to see the doppler effect in this case.

### 4) AMC
```A_amc.m```

We wanted to try to create an adaptive modulation coding algorithm based on the sinr tables. We have a UE1 that is going far away from the BS, so that as the sinr decreases the MC changes accordingly. So we should see the constellation changing.

### 5) altro?
```.m```


# PLOTS TO COMPARE:
* SINR vs TIME
* BER vs TIME
* THROUPUT vs TIME
* MODULATION ORDER vs TIME (in A.4)
* DOPPLER vs TIME (in A.3)


# Christmas BONUS:
## S: 3D, 1BS + 1 BS mobile (drone), 3GPP, OFDM/NR, LMS (wideband)
```bonus.m```






