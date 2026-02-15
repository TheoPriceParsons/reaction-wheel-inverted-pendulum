\documentclass[11pt]{article}

% --------------------
% Packages
% --------------------
\usepackage[margin=1in]{geometry}
\usepackage{amsmath, amssymb, amsfonts}
\usepackage{physics}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{enumitem}
\usepackage{hyperref}
\usepackage{cleveref}

% --------------------
% Title Information
% --------------------
\title{\textbf{Nonlinear Modeling, Control, and Estimation of an Underactuated Pendulum System}}
\author{Theodore Price-Parsons}
\date{\today}

% --------------------
% Document
% --------------------
\begin{document}

\maketitle

\begin{abstract}
% Abstract intentionally left blank.
\end{abstract}

% --------------------------------------------------
\section{Problem Statement}

\subsection{System Description}
Describe the physical system being studied.

\subsection{Objective}
Develop a first-principles dynamic model of a reaction-wheel-actuated pendulum using Lagrangian mechanics, derive a state-space representation suitable for control design, and implement a controller to stabilize the pendulum in its upright (unstable) equilibrium.
I'll write ts later...
\subsection{Why the Problem Is Nontrivial}
Explain sources of difficulty (e.g., nonlinearities, constraints, uncertainty).

% --------------------------------------------------
\section{Physical Model}

\documentclass[11pt]{article}

% --------------------
% Packages
% --------------------
\usepackage[margin=1in]{geometry}
\usepackage{amsmath, amssymb, amsfonts}
\usepackage{physics}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{enumitem}
\usepackage{hyperref}
\usepackage{cleveref}

% --------------------
% Title Information
% --------------------
\title{\textbf{Nonlinear Modeling, Control, and Estimation of an Underactuated Pendulum System}}
\author{Theodore Price-Parsons}
\date{\today}

% --------------------
% Document
% --------------------
\begin{document}

\maketitle

\begin{abstract}
% Abstract intentionally left blank.
\end{abstract}

% --------------------------------------------------
\section{Problem Statement}

\subsection{System Description}
Describe the physical system being studied.

\subsection{Objective}
Develop a first-principles dynamic model of a reaction-wheel-actuated pendulum using Lagrangian mechanics, derive a state-space representation suitable for control design, and implement a controller to stabilize the pendulum in its upright (unstable) equilibrium.
I'll write ts later...
\subsection{Why the Problem Is Nontrivial}
Explain sources of difficulty (e.g., nonlinearities, constraints, uncertainty).

% --------------------------------------------------
\section{Physical Model}

\subsection{Assumptions}
\begin{itemize}[leftmargin=2em]
    \item Massless and rigid pendulum
    \item No friction and no air resistance
    \item Planar motion
    \item Perfect motor
    \item Gravity $g$ is constant 
\end{itemize}

\subsection{Coordinates}
Define generalized coordinates and reference frames.

\subsection{Parameters}
List of symbolic parameters and their physical meanings.

\begin{center}
\begin{tabular}{ll}
\toprule
Symbol & Description \\
\midrule
$\theta$ & Pendulum angle measured from the upward vertical \\
$\phi$ & Reaction wheel angle relative to the pendulum \\
\bottomrule
\end{tabular}
\end{center}

The unstable equilibrium of interest corresponds to
\[
\theta = 0 .
\]

The absolute angular position of the reaction wheel is given by
\[
\theta + \phi .
\]


% --------------------------------------------------
\section{Equations of Motion}
\section{Kinetic Energy}


In our model we have two sources of Kinetic energy.


\subsection{Pendulum mass (translational)}

$$T_{\text{pend}} = \frac{1}{2} m l^2 \dot{\theta}^2$$


\subsection{Wheel kinetic energy (rotational)}

$$T_{\text{wheel}} = \frac{1}{2} I_w (\dot{\theta} + \dot{\phi})^2$$

\subsection{Total kinetic energy}

$$T = \frac{1}{2} m l^2 \dot{\theta}^2
  + \frac{1}{2} I_w (\dot{\theta} + \dot{\phi})^2$$

\subsection{Potential Energy}

$$V = m g l \cos\theta$$
\subsection{Lagrangian}
The Lagrangian is defined as
\begin{equation}
\mathcal{L} = T - V
\end{equation}

In our case we have:

$$\mathcal{L}
= \frac{1}{2} m l^2 \dot{\theta}^2
+ \frac{1}{2} I_w (\dot{\theta} + \dot{\phi})^2
- m g l \cos\theta
$$

\subsection{Equations of Motion}
Using the Euler--Lagrange equations,

\begin{equation}
\frac{d}{dt}\left( \frac{\partial \mathcal{L}}{\partial \dot{q}_i} \right)
- \frac{\partial \mathcal{L}}{\partial q_i}
= Q_i
\end{equation}
where $Q$ represents generalized forces. In our case it will be:

$$Q_\theta = -\tau$$
$$Q_\phi = +\tau$$


All these equations lead to the following.

For $\theta$:

$$(m l^2 + I_w)\ddot{\theta}
+ I_w \ddot{\phi}
+ m g l \sin\theta
= -\tau
$$

For $\phi$:

$$I_w(\ddot{\theta} + \ddot{\phi}) = \tau$$


\subsection{Linearization}


At equilibrium we have: 
$$\theta=0, \phi=0, \dot{\phi}=0,\dot{\theta}=0 $$

We want to linearize the only term that is not linear, so $sin(\theta)$ we use the small angle approximation $sin(\theta) \approx \theta$.


Which gives us: 



$$(m l^2 + I_w)\ddot{\theta}
+ I_w \ddot{\phi}
+ m g l \theta
= -\tau
$$

$$I_w(\ddot{\theta} + \ddot{\phi}) = \tau$$


We then solve for $\ddot{\theta}$ and $\ddot{\phi}$.
\begin{align}
\ddot{\theta}
&= -\frac{g}{l}\,\theta
- \frac{2}{m l^2}\,\tau
\\[6pt]
\ddot{\phi}
&= \frac{1}{I_w}\,\tau
+ \frac{g}{l}\,\theta
+ \frac{2}{m l^2}\,\tau
\end{align}


% --------------------------------------------------
\section{State-Space Formulation}

\subsection{Nonlinear Model}
Let the state vector be
\[
x =
\begin{bmatrix}
\theta \\
\dot{\theta} \\
\dot{\phi}
\end{bmatrix}.
\]

We have the equation the system of linear DEs:


$$\dot{x}=Ax+B \tau$$
We define the state variables as
\[
x_1 = \theta, \quad
x_2 = \dot{\theta}, \quad
x_3 = \dot{\phi}.
\]

We then derive the equations: 


\begin{align}
\dot{x}_1 &= x_2, \\
\dot{x}_2 &= -\frac{g}{l}x_1 - \frac{2}{m l^2}\tau, \\
\dot{x}_3 &= \frac{g}{l}x_1
+ \left(\frac{1}{I_w} + \frac{2}{m l^2}\right)\tau.
\end{align}

Which becomes:


\[
\dot{x}
=
\begin{bmatrix}
0 & 1 & 0 \\
-\dfrac{g}{l} & 0 & 0 \\
\dfrac{g}{l} & 0 & 0
\end{bmatrix}
x
+
\begin{bmatrix}
0 \\
-\dfrac{2}{m l^2} \\
\dfrac{1}{I_w} + \dfrac{2}{m l^2}
\end{bmatrix}
\tau
\]
\subsection{Feedback}

We have our feedback law, we want to apply torque in a way that's proportional to how wrong each state is: 

$$\tau=-Kx$$

We now want to find this K.


% --------------------------------------------------
\section{Planned Control \& Estimation Approach}

\subsection{Controller Candidates}
Potential control strategies include:
\begin{itemize}[leftmargin=2em]
    \item Linear Quadratic Regulator (LQR)
    \item Feedback linearization
    \item Nonlinear MPC
\end{itemize}
In this case we use LQR since at this point we are not handling constraints but also because we are near a linearized equilibrium. Added to the fact that it is robust and the choices of Q and R are an easy way to set our priorities.

(for values of K and other simulations see jupyter report) 


\subsection{Up-Swing}

Since the LQR method we added only works around the equilibrium position and does not work with large angles. So we must use a different approach for the swing up part of this: 
\subsection{Estimator Candidates}
Potential state estimation approaches include:
\begin{itemize}[leftmargin=2em]
    \item Extended Kalman Filter (EKF)
    \item Unscented Kalman Filter (UKF)
    \item Luenberger observer
\end{itemize}

% --------------------------------------------------
\end{document}
