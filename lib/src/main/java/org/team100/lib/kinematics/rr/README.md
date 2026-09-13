# RR Kinematics

Revolute - Revolute

A 2-DOF arm, as in 2023.

<img src="../../dynamics/rr/image_rr.png">

Cartesian joint positions are $P_1$ (the elbow) and $P_2$ (the end-effector).

Joint configurations are $q_1$, relative to the x axis,
and $q_2$, measured relative to the proximal link.

Link lengths are $l_1$ and $l_2$.

```math
\begin{align}
s_i &= sin(q_i)\\
c_i &= cos(q_i) \\
s_{ij} &= sin(q_i + q_j)\\
c_{ij} &= cos(q_i + q_j)
\end{align}
```

There are two implementations here:

* one analytic using the law of cosines
* one using the product-of-exponentials method from Modern Robotics (really just as an exercise)



## Forward kinematics

<img src="image.png" width=600/>

```math
P_1 = 
\begin{bmatrix}
x_1\\
y_1
\end{bmatrix}
=
\begin{bmatrix}
l_1 c_1\\
\\
l_1 s_1
\end{bmatrix}
```
```math
P_2 =
\begin{bmatrix}
x_2\\
y_2
\end{bmatrix}
=
\begin{bmatrix}
x_1 + l_2 c_{12}\\
\\
y_1 + l_2 s_{12}
\end{bmatrix}
=
\begin{bmatrix}
l_1 c_1 + l_2 c_{12}\\
\\
l_1 s_1 + l_2 s_{12}
\end{bmatrix}
```


## Inverse kinematics

There are up to two "postures" for a given position, generally called "elbow up" and
"elbow down".  

```math
r = \|P2\|
```
```math
\gamma = atan2(y_2, x_2)
```
```math
\beta = \pm \: acos\left(\frac{r^2 + l_1^2 - l_2^2}{2  r  l_1}\right)
```
```math
\alpha = acos\left(\frac{l_1^2  + l_2^2 - r^2} {2 * l_1 * l_2}\right)
```

```math
q_1 = \gamma + \beta
```
```math
q_2 = \alpha + \pi
```

## Velocity

the function for cartesian velocity, $\dot{x}$, can be written
in terms of the Jacobian, $J$:

```math
\begin{equation}
\dot{x} = J \dot{q}
\end{equation}
```

To obtain the Jacobian, differentiate the forward kinematics:

Using end-effector coordinates $(x,y)$:

```math
J = 
\begin{bmatrix}
\frac{\partial x}{\partial q_1} & \frac{\partial x}{\partial q_2} \\
\\
\frac{\partial y}{\partial q_1} & \frac{\partial y}{\partial q_2} \\
\end{bmatrix}
```

substituting:

```math
J =
\begin{bmatrix}
\frac{\partial}{\partial q_1} (l_1 c_1 + l_2 c_{12}) &
\frac{\partial}{\partial q_2} (l_1 c_1 + l_2 c_{12}) \\
\\
\frac{\partial}{\partial q_1} (l_1 s_1 + l_2 s_{12}) &
\frac{\partial}{\partial q_2} (l_1 s_1 + l_2 s_{12}) \\
\end{bmatrix}
```

and so:

```math
J =
\begin{bmatrix}
 -l_1 s_1 - l_2 s_{12} && - l_2 s_{12} \\
 \\
l_1 c_1 + l_2 c_{12} && l_2 c_{12} \\
\end{bmatrix}
```


## Acceleration

To find the acceleration, differentiate again and apply the product rule:

```math
\begin{equation}
\ddot{x} = \dot{J}\dot{q} + J\ddot{q}
\end{equation}
```


Computing the time derivative of the Jacobian is simple enough, remembering
to apply the chain rule:

```math
\begin{equation}
\dot{J} =
\begin{bmatrix}
-l_1 c_1 \dot{q_1} - l_2 c_{12}(\dot{q_1}+\dot{q_2}) 
&&
-l_2c_{12}(\dot{q_1}+\dot{q_2}) \\
\\
-l_1s_1\dot{q_1}-l_2s_{12}(\dot{q_1}+\dot{q_2})
&& 
-l_2s_{12}(\dot{q_1}+\dot{q_2})
\end{bmatrix}
\end{equation}
```


To find the joint accelerations from the cartesian requires only the inverse of
the Jacobian itself, not the inverse of the derivative:

```math
\begin{equation}
\ddot{q} = J^{-1}(\ddot{x} - \dot{J}\dot{q})
\end{equation}
```

Or

```math
\begin{equation}
\ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
\end{equation}
```

## References 

* [Cornell text](https://rpal.cs.cornell.edu/foundations/kinematics.pdf)