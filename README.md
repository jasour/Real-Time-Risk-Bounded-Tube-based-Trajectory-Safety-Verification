# Real-Time Risk-Bounded Tube-based Trajectory Safety Verification
CDC 2021: Real-Time Risk-Bounded Tube-based Trajectory Safety Verification

**Risk:** Given the uncertain safety constraint X(t,w), e.g., randomly moving obstacle with uncertain size and location, and the planned state trajectory P(t), we define the risk at time t as the probability of violation of the uncertain safety constraints by the given trajectory
P(t) at time t. 

**Risk-Aware Safety Verification Problem:** In the risk-aware safety verification problem, we want to make sure that the probability of violation of the
uncertain safety constraints, i.e., risk, is bounded over the entire planning time horizon [t0, tf ]. More precisely, we aim at
verifying the following probabilistic safety constraints:
<p align="center">
Probability (P(t) violates uncertain safet constraint X(t,w)  ) <= ∆  for all t in [t0, tf]
<p>
where w is uncertain vector with known probability distribution, randomly moving obstacle with uncertain size and location.

**Tube-Based Risk-Aware Safety Verification Problem:** Due to the external disturbances, states of the autonomous systems tend to deviate from the planned trajectory. Hence, to ensure the safety, we want to make sure that all the trajectories in the neighborhood of the planned trajectory are also safe. For this purpose, we use tubes to represent the family of the trajectories in the neighborhood of the planned trajectory. In the tube-based risk-aware safety verification problem, we want to make sure that the probability of violation of the uncertain safety constraints by the given tube Tube(P(t)) is bounded. More precisely, we aim at verifying the following probabilistic safety constraints:

<p align="center">
Probability (Tube(P(t)) violates uncertain safet constraint X(t,w)  ) <= ∆  for all t in [t0, tf]
<p>
where Tube(P(t)) is the defined tube around the trajectory P(t) and w is uncertain vector with known probability distribution, randomly moving obstacle with uncertain size and location.


<p align="center">
<img src="https://github.com/jasour/Real-Time-Risk-Bounded-Tube-based-Trajectory-Safety-Verification/blob/main/Img/Tube.png" width="500" height="300" />
<p align = "center">
<p align="center">
Tube around the planned trajectory. In the tube-based safety verification problem, we want to make sure that tube satisfies the probabilistic safety
constraints.
<p align = "center">

**Application:** Tube-based risk-aware safety verification can be integrated into standard and deep-learning-based planners to verify the safety of the generated plans of autonomous systems in the presence of uncertainties.

<p align="center">
<img src="https://github.com/jasour/Real-Time-Risk-Bounded-Tube-based-Trajectory-Safety-Verification/blob/main/Img/Risk-Aware-Planning.png"  />
<p align = "center">
<p align="center">
Tube-based Risk-Aware Safety Layer for Planning
<p align = "center">
