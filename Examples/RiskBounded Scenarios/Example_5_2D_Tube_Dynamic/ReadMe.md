

- Probabilistic Time-Varying Safety Constraint (safe region): Moving obstacle with uncertain size and location

<p align="center">
{(x1,x2,t) : w1^2-((x1-2-0.1*w2)/1)^2-((x2-5+t/1.5-0.1*w3)/2)^2 >= 0 }
<p>
where w1 is the uncertain radius with Uniform distribution on [0.3, 0.4], w2 has normal distribution N(0,0.1), and w3 has Beta distribution Beta(3,3).

- Acceptable Risk Level: ∆ = 0.01

- Trajectory: (Px(t),Py(t))  where Px(t)=t, Py(t)=((t-5)^4 + 2*(t-5)^3 - 15*(t-5)^2 - 12*(t-5) + 36)/20

- Tube Size: 0.4

- Start and final time: t in [t0=0, tf=9]

<p align="center">
<img src="https://github.com/jasour/Real-Time-Risk-Bounded-Tube-based-Trajectory-Safety-Verification/blob/main/Examples/RiskBounded%20Scenarios/Example_5_2D_Tube_Dynamic/plot.png" width="900" height="500" />
<p align = "center">
<p align="center">
Given Trajectory, Tube, and Risk Contour of the Probabilistic Moving Obstacle. 
Probability of collision with the uncertain obstacle outside of the red curve is less or equal to ∆=0.01. Yellow ellipse shows that the tube at current time is safe.<p align = "center">

