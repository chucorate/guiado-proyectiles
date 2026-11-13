# Modelamiento Lineal

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

### Resultado Analítico

Ahora definimos la función de costo de nuestro PLC de la forma

$$
J(u) := z(T)^TQz(T) + \int_0^T u^2(t) dt,
$$

donde la integral penaliza el esfuerzo de control y el término terminal penaliza la separación vertical al finalizar, donde $Q=\mathrm{diag}(q_f,0)$. El parámetro $q_f>0$ es el peso de asociado a la penalización en $r_y(T)\neq0$. Se deja este elemento de la diagonal como una variable ya que es una restricción "suave" sobre la distancia final entre el objetivo y el misil, por lo que queremos encontrar una solución en función de $q_f$ para luego $q_f \to \infty$

Además de esto el tiempo de encuentro, por lo que se dijo antes sobre ignorar las componentes en $x$ del problema, está dado por $T_{\mathrm{hit}}=-r_{x,0}/v_{x,0}>0$, ya que el objetivo no tiene aceleración y el misil tiene solamente aceleración tangencial a la LOS, de modo que en la componente $x$ se tiene $r_x(t)=r_{x,0}+v_{x,0}t$ con $r_{x,0}>0$ y $v_{x,0}<0$.

Definimos además $t_{\mathrm{go}}=T_{\mathrm{hit}}-t$ como el tiempo que falta para ese encuentro, de modo que $t_{\mathrm{go}}>0$ para $0\leq t<T_{\mathrm{hit}}$ y tiende a cero al aproximarse al tiempo final.

Luego, como este problema tiene un control en dimensión 1 se cumple trivialmente la condición de exsitencia y unicidad para el PLC por lo que podemos utilizar la solución de feedback lineal Riccati

$$
u(t) = B^TE(t)z(t)
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
u_{q_f}(t) = \frac{3}{t^2_\mathrm{go}}\left[ \frac{r_{y,t}+v_{y,t}\cdot t_\mathrm{go}}{1+\frac{3}{q_ft^3_{\mathrm{go}}}} \right]
$$

Y para poder tener una restricción dura como se dijo antes con $q_f \to \infty$ tenemos que 

$$
u_\infty(t) = \frac{3}{t^2_\mathrm{go}}[r_{y,t}+v_{y,t}\cdot t_\mathrm{go}]
$$


















