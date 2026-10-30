# Resultado Analítico
Para trabajar con un sistema lineal, consideraremos $a_{T}=0$. En dicho caso, del sistema general se deduce que

$$
\begin{align}\frac{d\vec{r}}{dt}= \vec{v},\qquad 
\frac{d\vec{v}}{dt}=-u\langle\hat r,\hat v_M\rangle J\hat v_M,\qquad\frac{d\vec{v}_{T}}{dt}= 0,
\end{align}
$$

Dado que $\langle\hat r,\hat v_M\rangle=\cos L$, si expresamos la dinámica en función de las componentes de $\vec r=(r_x,r_y)$ y $\vec v=(v_x,v_y)$, obtenemos $\dot r_x=v_x,\ \dot r_y=v_y$, y además

$$
\begin{align*}
\frac{d}{dt}\begin{pmatrix}v_{x} \\ v_{y}\end{pmatrix} &=-u\cos(L)J\hat v_M\\
&=-u\cos (L)\begin{pmatrix}0&-1\\ 1&0\end{pmatrix}
\begin{pmatrix}\cos\theta_M\\ \sin\theta_M\end{pmatrix}\\
&=u\cos (L)\begin{pmatrix}\sin\theta_M\\ -\cos\theta_M\end{pmatrix}
\end{align*}
$$

Asumimos además que $\lambda$ y $\theta_M$ son pequeños, de modo que $L$ también lo es. Así, $\cos L\approx1$, $\cos\theta_M\approx1$ y $\sin\theta_M\approx\theta_M$. 

Al linealizar alrededor de una trayectoria rectilínea con $\lambda\approx\theta_M\approx u \approx 0$, el término $u\theta_M$ es de segundo orden, por lo que se obtiene

$$
\dot v_x\approx0,\qquad \dot v_y\approx-u.
$$

Luego podemos expresar la dinámica linealizada como

$$
\frac{d}{dt}\begin{pmatrix}r_{x } \\  r_{y} \\  v_{x} \\  v_{y}\end{pmatrix}=\begin{pmatrix}0&0&1& 0 \\  0&0&0&1 \\  0&0&0&0 \\ 0&0&0&0\end{pmatrix}\begin{pmatrix}r_x  \\  r_{y} \\  v_{x} \\  v_{y}\end{pmatrix}+\begin{pmatrix}0 \\  0 \\  0 \\  -1\end{pmatrix}u
$$

Adicionalmente, asumimos que la velocidad de acercamiento $V_c=-\dot R=-\langle \vec{v}, \hat{r} \rangle$ es positiva. Como $\lambda$ es pequeño, tenemos que

$$
V_c=-v_x\cos\lambda-v_y\sin\lambda\approx-v_x -\lambda v_{y}\approx-v_{x}>0,
$$

de modo que el proyectil siempre se acerca en la dirección $x$ al target, por lo que basta controlar la dinámica en la componente $y$. Definiendo $z=(r_{y},v_{y})$, el sistema linealizado anterior se puede reducir a 

$$
\frac{dz}{dt}=\underbrace{ \begin{pmatrix}0&1 \\  0&0 \end{pmatrix} }_{ A }z+\underbrace{ \begin{pmatrix}0 \\  -1\end{pmatrix} }_{ B }u,\qquad z(0)=\begin{pmatrix}r_{y,0} \\  v_{y,0}\end{pmatrix}.
$$

Ahora definimos la función de costo de nuestro PLC de la forma

$$
J(u) := x(T)^TQx(T) + \int_0^T u^2(t) dt  
$$

donde no hay costo para el recorrido ya que asumimos condiciones ideales sin roce ni factores similares, un costo costo cuadratico para el control (que es común en este tipo de problemas ya que al ser el factor con el que se controla la aceleración esta integral es un simil a la energía) y $Q$ de la forma $diag(q_f, 0)$ ya que queremos penalizar $r_{y,T}\neq 0$ pero no nos importa la velocidad a la que ocurre el encuentro o los terminos cruzados de la relación velocidad posición.

Se deja este elemento de la diagonal como una variable ya que es una restricción "suave" sobre la distancia final entre el objetivo y el misil por lo que queremos encontrar una solución en función de $q_f$ para luego $q_f \to \infty$

Además de esto el tiempo de encuentro, por lo que se dijo antes sobre ignorar las componentes en $x$ del problema está dado por $T_{hit} = r_{x,0}/v_{x,0}$ ya que el objetivo no tiene aceleración y el misil tiene solamente aceleración tangencial a la LOS por lo que al ser angulos pequeños no influye en la velocidad del eje $y$

Luego, como este problema tiene un control en dimensión 1 se cumple trivialmente la condición de exsitencia y unicidad para el PLC por lo que podemos utilizar la solución de feedback lineal Riccati

$$
u(t) = B^TE(t)x(t)
$$

Donde $E(t)$ cumple la dinámica de Riccati:

$$
\begin{align}
\dot{E}(t) &= -A^TE(t)-E^T(t)A-E^T(t)BB^TE(t)\\
E(T_{hit}) &= -Q
\end{align}
$$

Ahora, basándonos en el texto [Modern Homing Missile Guidance Theory and
Techniques, Johns Hopkins APL Technical Digest]() para este problema se da una solución analítica del control dada por

$$
u_{q_f}(t) = \frac{3}{t^2_{go}}\left[ \frac{r_{y,t}+v_{y,t}\cdot t_{go}}{1+\frac{3}{q_ft^3_{go}}} \right]
$$

Y para poder tener una restricción dura como se dijo antes con $q_f \to \infty$ tenemos que 

$$
u_\infty(t) = \frac{3}{t^2_{go}}[r_{y,t}+v_{y,t}\cdot t_{go}]
$$


















