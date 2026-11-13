## Modelamiento de dos proyectiles acoplados

En el caso de un sólo proyectil, de los modelamientos anteriores tenemos que la velocidad de acercamiento está dada por $V_{c}=-\dot{R}$ con $R$ la magnitud de la LOS. Al considerar una aproximación lineal de las trayectorias, definiendo $y=y_{T}-y_{M}$ la distancia vertical relativa entre el proyectil y el objetivo, tenemos que la dinámica está dada por 

$$
\frac{d}{dt}\begin{pmatrix}y \\  \dot{y} \end{pmatrix}=\begin{pmatrix}0 & 1 \\  0 & 0\end{pmatrix}\begin{pmatrix}y \\  \dot{y}\end{pmatrix}+\begin{pmatrix}0 \\  1\end{pmatrix}u,
$$

donde $u$ controla la aceleración normal del proyectil. La deducción es análoga a como se hizo en el caso lineal de un proyectil, solo que ahora se usa la convención de que $u$ tenga signo positivo en lugar de negativo para poder usar los resultados del paper [Closed-Form Cooperative Guidance Law For Two Missiles With Coupled Terminal Velocity Constraints](https://www.icas.org/icas_archive/ICAS2016/data/papers/2016_0387_paper.pdf).

En el caso de dos misiles, consideramos los estados $(y_1,\dot{y}_1)$ para primer proyectil, y $(y_2,\dot{y}_2)$ para el segundo, de manera que el estado completo está dado por 

$$
x(t)=\Big(y_{1}(t),\dot{y_{1}}(t),y_{2}(t),\dot{y}_{2}(t)\Big).
$$

Aquí, los proyectiles son independientes, por lo que podemos aplicarles la dinámica anterior a cada uno bajo las mismas suposiciones anteriores. Así, la dinámica del sistema está dada por

$$
\frac{d}{dt}\begin{pmatrix}y_{1} \\  \dot{y}_{1}  \\  y_{2} \\  \dot{y}_{2}\end{pmatrix}=\underbrace{ \begin{pmatrix}0 & 1 & 0 & 0 \\  0 & 0 & 0 & 0 \\  0 & 0 & 0 & 1 \\  0 & 0 & 0 & 0 \end{pmatrix} }_{ A }\begin{pmatrix}y_{1} \\  \dot{y_{1}} \\  y_{2} \\  \dot{y}_{2}\end{pmatrix}+\underbrace{ \begin{pmatrix}0 & 0 \\  1 & 0 \\  0 & 0\\  0 & 1\end{pmatrix} }_{ B }\begin{pmatrix}u_{1} \\  u_{2}\end{pmatrix},
$$

donde $u_1$ y $u_2$ controlan la aceleración normal del primer y segundo proyectil respectivamente. 

Si consideramos que la rapidez de cada proyectil satisface $V_{m_1} = V_{m_2}=V_m$, obtenemos la relación del ángulo de impacto deseado $\theta$ como

$$
\theta = \theta_{m_1} - \theta_{m_2} \approx \frac{\dot{y}_1-\dot{y}_2}{V_m}.
$$

Definiendo el parámetro $r=V_m\theta$, el funcional de costo que penaliza tanto los esfuerzos de los controles como la lejanía del ángulo de referencia en el impacto, está dado por 

$$
J(u)=\frac{b}{2}(y_{1}^{2}(T)+y_{2}^{2}(T))+\frac{c}{2}(\dot{y}_{1}(T)-\dot{y}_{2}(T)-r)^{2}+\frac{1}{2}\int_{t_{0}}^{T} u_{1}^{2}(t)+u_{2}^{2}(t) \, dt,
$$

donde $b>0$ es un parámetro que mide qué tanto se penalizan los errores entre posiciones relativas, y $c>0$ es un parámetro que penaliza el error de las velocidades terminales en relación a la diferencia $r$ deseada.

## Soluciones analíticas

En base al [paper mencionado](https://www.icas.org/icas_archive/ICAS2016/data/papers/2016_0387_paper.pdf), se desprende que para el sistema anterior el control óptimo está dado por 

$$
\begin{pmatrix}u_{1}(t) \\  u_{2}(t)\end{pmatrix}=-\begin{pmatrix}by_{1}(T)\tau+c(\dot{y}_{1}(T)-\dot{y}_{2}(T)-r) \\  by_{2}(T)\tau+c(\dot{y}_{1}(T)-\dot{y}_{2}(T)-r) \end{pmatrix},
$$

donde $\tau = T - t$. Para expresar esta ley de control en función del estado actual $x(t)$ y no del estado final $x(T)$, se usa la relación

$$
x(T) = \Phi(T, t)x(t) + \Psi(t) r,
$$

donde la matriz de transición $\Phi(T, t)$ y el término afín $\Psi(t)$ están dadas por

$$
\Phi(T, t) = (I - A')^{-1}B', \qquad \Psi(t) = (I - A')^{-1}S,
$$

con

$$
\begin{align*}
A' &= \begin{pmatrix}
-\frac{1}{3}b\tau^3 & -\frac{1}{2}c\tau^2 & 0 & \frac{1}{2}c\tau^2 \\
-\frac{1}{2}b\tau^2 & -c\tau & 0 & c\tau \\
0 & \frac{1}{2}c\tau^2 & -\frac{1}{3}b\tau^3 & -\frac{1}{2}c\tau^2 \\
0 & c\tau & -\frac{1}{2}b\tau^2 & -c\tau
\end{pmatrix}, \\
B' &= \begin{pmatrix}
1 & \tau & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & \tau \\
0 & 0 & 0 & 1
\end{pmatrix}, \\
S &= \begin{pmatrix}
\frac{1}{2}c\tau^2 \\
c\tau \\
-\frac{1}{2}c\tau^2 \\
-c\tau
\end{pmatrix}.
\end{align*}
$$

Así, el control se puede escribir como una combinación lineal de $x(t)$ y $r$ de la forma

$$
u(t)=-(K(t)x(t)+V(t)r),
$$

donde $K(t)\in\mathbb{R}^{2\times4}$ es una matriz de ganancia y $V(t)\in \mathbb{R}^2$ es un vector que representa el término afín asociado al ángulo de referencia, dados por

$$
\begin{align*}
K(t) &= \begin{pmatrix} b\tau & c & 0 & -c \\ 0 & -c & b\tau & c \end{pmatrix} (I - A')^{-1} B', \\[6pt]
V(t) &= \begin{pmatrix} b\tau & c & 0 & -c \\ 0 & -c & b\tau & c \end{pmatrix} (I - A')^{-1} S + \begin{pmatrix} -c \\ c \end{pmatrix}.
\end{align*}
$$
