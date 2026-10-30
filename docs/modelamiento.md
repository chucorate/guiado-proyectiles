# Modelamiento de Persecución Target-Proyectil

[Referencia](https://secwww.jhuapl.edu/techdigest/content/techdigest/pdf/V29-N01/29-01-Palumbo_Homing.pdf)

## Modelo General 2D

El Target $T$ sigue la dinámica

$$
\begin{align}
\frac{d\vec{v}_{T}}{dt} &=  \vec{a}_{T} ,\qquad \vec{v}_{T}(0)=\vec{v}_{T,0},\\
\frac{d\vec{x}_{T}}{dt} &= \vec{v}_{T},\qquad \vec{x}_{T}(0)=\vec{x}_{T,0}.
\end{align}
$$

Sea $\vec{a}_{T}=a_{v}\hat{v}_{T}+a_{n} \hat{n}$ , donde $\hat{v}_{T}$ es la dirección unitaria de la velocidad, y $\hat{n}$ es la dirección unitaria perpendicular a $\hat{v}_{T}$. Si consideramos la rotación de 90° 

$$
J=\begin{pmatrix}0 & -1 \\  1 & 0\end{pmatrix},
$$

entonces $\hat{n}=J\hat{v}_{T}$. Para que la aceleración sea perpendicular al movimiento, la componente de aceleración en $\hat{v}_{T}$ debe ser nula, por lo que 

$$
\vec{a}_{T}=a_{T}\hat{n}=a_{T}J\hat{v}_{T}=a_{T}J\frac{\vec{v}_{T}}{\lVert \vec{v}_{T} \rVert }.
$$

Por otro lado, el proyectil obedece a la dinámica dada por

$$
\begin{align}
\frac{d\vec{v}_{M}}{dt} &=  \vec{a}_{M} ,\qquad \vec{v}_{M}(0)=\vec{v}_{M,0},\\
\frac{d\vec{x}_{M}}{dt} &= \vec{v}_{M} ,\qquad \vec{x}_{M}(0)=\vec{x}_{M,0}.
\end{align}
$$

En este caso, la aceleración $a_{M}$ es perpendicular a la LOS. Denotando $\vec{r}=\vec{x}_{T}-\vec{x}_{M}$, la LOS está dada por 

$$
\hat{r}=\frac{\vec{r}}{\lVert \vec{r} \rVert }.
$$

Así, su aceleración está dada por 

$$
\vec{a}_{M}=a_{M}J\hat{r}.
$$

Dado que esta aceleración es perpendicular a la LOS y no a la dirección de movimiento del misil, la velocidad $\vec{v}_M$ a priori no es constante. Para ello, debemos considerar sólo la proyección de $\vec{a}_M$ perpendicular a $\vec{v}_M$. 

Sea $\hat{v}_M = \frac{\vec{v}_{M}}{\lVert \vec{v}_{M} \rVert }$. Al desarrollar la componente $J\hat{v}_{M}$ perpendicular a la velocidad, se llega a

$$
\begin{align*}
\vec{a}^{\perp}_{M}&= \langle \vec{a}_{M}, J\hat{v}_{M} \rangle J\hat{v}_{M}  \\
&= a_{M}\langle J\hat{r}, J\hat{v}_{M} \rangle J\hat{v}_{M} \\
&= a_{M}\left( \hat{r}^{\intercal}J^{\intercal}J\hat{v}_{M} \right)J\hat{v}_{M} \\
&=a_{M}\left( \hat{r}^{\intercal}\hat{v}_{M} \right)J\hat{v}_{M} \\
&= a_{M}\langle \hat{r},\hat{v}_{M}  \rangle J\hat{v}_{M}  
\end{align*}
$$

De esta forma, podemos hacer que la aceleración del proyectil sólo use esta componente para desplazarse, es decir que reasignaremos el valor de la aceleración del proyectil como

$$\vec{a}_{M}\leftarrow\vec{a}^{\perp}_{M} = a_{M}\langle \hat{r},\hat{v}_{M}  \rangle J\hat{v}_{M},$$

de modo que 

$$
\frac{d}{dt}\lVert v_{M}(t) \rVert ^{2}=2\langle v_{M}(t), \dot{v} _{M}(t)\rangle =2a_{M}\langle \hat{r}, \hat{v} _{M}\rangle \underbrace{ \langle v_{M}(t), J\hat{v}_{M}(t) \rangle }_{ 0 } =0
$$
y por lo tanto $\lVert v_M(t) \rVert=V_M$ es constante.

Con todo lo anterior, es posible expresar la dinámica como un sistema de la forma $\dot{z}(t)=f(z(t),u(t)),\ z(0)=z_{0}$. De la definición de $\vec{r}$, tenemos que 

$$
\frac{d\vec{r}}{dt}=\frac{d}{dt}\vec{x}_{T}-\frac{d}{dt}\vec{x}_{M}=\vec{v}_{T}-\vec{v}_{M}.
$$

Definiendo $\vec{v}=\vec{v}_{T}-\vec{v}_{M}$, se tiene que $\hat{v}_M=\frac{\vec{v}_{T}-\vec{v}}{\lVert \vec{v}_{T}-\vec{v} \rVert }$ y además

$$
\frac{d \vec{v}}{dt}=\frac{d}{dt}\vec{v}_{T}-\frac{d}{dt}\vec{v}_{M}=a_{T}J\hat{v}_{T}-a_{M}\langle \hat{r},\hat{v}_{M}  \rangle J\hat{v}_{M} 
$$

Así, el sistema se expresa de la forma

$$
\begin{align}\frac{d\vec{r}}{dt}&= \vec{v}, \qquad \qquad \qquad \qquad \qquad \qquad\quad\vec{r}(0)=\vec{x}_{T,0}-\vec{x}_{M,0}\\
\frac{d\vec{v}}{dt}&= a_{T}J\frac{\vec{v_T}}{\lVert \vec{v_T} \rVert }-a_{M}\langle \hat{r},\hat{v}_{M}  \rangle J\hat{v}_{M} , \qquad \vec{v}(0)=\vec{v}_{T,0}-\vec{v}_{M,0}\\
\frac{d\vec{v}_{T}}{dt}&= a_{T}J\frac{\vec{v}_T}{\lVert \vec{v}_T \rVert }, \qquad \qquad\qquad  \qquad\quad \ \ \vec{v}_{T}(0)=\vec{v}_{T,0}
\end{align}
$$

donde el objetivo a un tiempo final $t_{f}$ es que $\vec{r}(t_{f})=0$, indicando que el proyectil colisionó con el target. Adicionalmente, dado que vamos a controlar cómo cambia la aceleración perpendicular del proyectil, se denotará $u(t):= a_{M}(t)$.

## Modelo en función de ángulos

Sean $\lambda$ el ángulo de la LOS, $\theta$ el ángulo de la velocidad relativa, $\theta_M$ el ángulo de movimiento del proyectil y $\theta_T$ el del objetivo, todos respecto al eje horizontal. Al definir las magnitudes $R=\lVert\vec r\rVert$, $V=\lVert\vec v\rVert$ y $V_T=\lVert\vec v_T\rVert$, tenemos que

$$  
\vec r=R\begin{pmatrix}\cos\lambda\\ \sin\lambda\end{pmatrix},\quad  
\vec v=V\begin{pmatrix}\cos\theta\\ \sin\theta\end{pmatrix},\quad  
\vec v_T=V_T\begin{pmatrix}\cos\theta_T\\ \sin\theta_T\end{pmatrix}.
$$

Además, podemos expresar las direcciones $\hat r$, $\hat v$, $\hat v_T$, $\hat v_M$ y sus rotaciones como

$$
\begin{align}\hat r&=\begin{pmatrix}\cos\lambda\\ \sin\lambda\end{pmatrix},&  
J\hat r&=\begin{pmatrix}-\sin\lambda\\ \cos\lambda\end{pmatrix},\\  
\hat v&=\begin{pmatrix}\cos\theta\\ \sin\theta\end{pmatrix},&  
J\hat v&=\begin{pmatrix}-\sin\theta\\ \cos\theta\end{pmatrix},\\
\hat v_T&=\begin{pmatrix}\cos\theta_T\\ \sin\theta_T\end{pmatrix},&  
J\hat v_T&=\begin{pmatrix}-\sin\theta_T\\ \cos\theta_T\end{pmatrix},\\
\hat v_M&=\begin{pmatrix}\cos\theta_M\\ \sin\theta_M\end{pmatrix},&
J\hat v_M&=\begin{pmatrix}-\sin\theta_M\\ \cos\theta_M\end{pmatrix}.\end{align}
$$

Derivando la expresión de $\vec{r}$ respecto al tiempo, obtenemos

$$
\frac{d\vec{r}}{dt}=\dot{R}\begin{pmatrix}\cos\lambda\\\sin\lambda\end{pmatrix}+R\dot{\lambda}\begin{pmatrix}-\sin\lambda\\\cos\lambda\end{pmatrix}.
$$

Como $\frac{d\vec{r}}{d{t}}=\vec{v}$, al proyectar sobre las direcciones $\hat{r}$ y $J\hat{r}$, obtenemos las ecuaciones

$$
\begin{align}
\left\langle\frac{d\vec r}{dt},\hat r\right\rangle  
=\langle\vec v,\hat r\rangle  
&\implies \dot R=V\cos(\theta-\lambda),\\
\left\langle\frac{d\vec r}{dt},J\hat r\right\rangle  
=\langle\vec v,J\hat r\rangle  
&\implies \dot\lambda=\frac{V}{R}\sin(\theta-\lambda).
\end{align}
$$

Por otro lado, derivando la velocidad del target, tenemos

$$
\frac{d\vec{v}_T}{dt}=\dot{V}_T\begin{pmatrix}\cos\theta_T\\\sin\theta_T\end{pmatrix}+V_T\dot{\theta}_T\begin{pmatrix}-\sin\theta_T\\\cos\theta_T\end{pmatrix}.
$$

Análogamente, al proyectar sobre las direcciones $\hat{v}_T$ y $J\hat{v}_T$, obtenemos

$$
\dot{V}_T=0,\qquad \dot{\theta}_T=\frac{a_T}{V_T}.
$$

De esta forma, la magnitud de la velocidad del objetivo se mantiene constante, mientras que su dirección varía según la aceleración perpendicular $a_T$.

Por último, para la velocidad relativa se tiene que

$$  
\frac{d\vec v}{dt}=\dot V\begin{pmatrix}\cos\theta\\ \sin\theta\end{pmatrix}+V\dot\theta\begin{pmatrix}-\sin\theta\\ \cos\theta\end{pmatrix}.  
$$

Definiendo $L:= \theta_M - \lambda$, se tiene que $\left\langle\hat r,  \hat v_M\right\rangle=\cos(\theta_M - \lambda) = \cos(L)$, y luego $\frac{d\vec v}{dt}=a_TJ\hat v_T-u\cos L\,J\hat v_M$. Al proyectar sobre las direcciones $\hat v$ y $J\hat v$, resulta

$$
\begin{align}  
\left\langle\frac{d\vec v}{dt},\hat v\right\rangle
&=\langle a_TJ\hat v_T-u\cos (L) J\hat v_M,\hat v\rangle\
&\implies \dot V=a_T\sin(\theta-\theta_T)-u\cos L\sin(\theta-\theta_M),\\
\left\langle\frac{d\vec v}{dt},J\hat v\right\rangle
&=\langle a_TJ\hat v_T-u\cos (L) J\hat v_M,J\hat v\rangle\
&\implies \dot\theta=\frac{a_T\cos(\theta-\theta_T)-u\cos L\cos(\theta-\theta_M)}{V}.
\end{align}  
$$

Con todo lo anterior, el sistema puede expresarse mediante las cinco variables de estado $z=(R,\lambda,V,\theta,\theta_T)$, con $V_T$ y $V_M$ constantes, obteniendo

$$
\begin{align}
\dot R&=V\cos(\theta-\lambda),\\ 
\dot\lambda&=\frac{V}{R}\sin(\theta-\lambda),\\  
\dot V&=a_T\sin(\theta-\theta_T)-u\cos L\sin(\theta-\theta_M),\\
\dot\theta&=\frac{a_T\cos(\theta-\theta_T)-u\cos L\cos(\theta-\theta_M)}{V},\\
\dot\theta_T&=\frac{a_T}{V_T}.
\end{align}
$$

para $R>0$, $V>0$, $V_T>0$ y $V_M>0$, donde $L=\theta_M-\lambda$ y $\theta_M$ se determina a partir de las variables de estado según la expresión anterior. Las condiciones iniciales se obtienen a partir de las posiciones y velocidades relativas iniciales del sistema general.