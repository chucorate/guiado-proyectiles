    ## Extensión general a múltiples proyectiles

La extensión está basada en el [planteamiento general](/docs/00_modelamiento.md). Consideremos ahora un sistema compuesto por $N$ proyectiles $M_i$, con $i\in I:=\{1,\ldots,N\}$, y un único objetivo $T$. Supondremos que el objetivo sigue una trayectoria independiente de los proyectiles y que estos no interactúan directamente entre sí.

Denotemos por $\vec{x}_{M_i}$ y $\vec{v}_{M_i}$ la posición y velocidad del $i$-ésimo proyectil, respectivamente. Análogamente al modelo general, definimos las variables relativas

$$
\vec{r}_i=\vec{x}_T-\vec{x}_{M_i},\qquad
\vec{v}_i=\vec{v}_T-\vec{v}_{M_i}.
$$

De esta forma, cada proyectil tiene su propia posición y velocidad relativa respecto al mismo objetivo. Derivando estas expresiones, obtenemos

$$
\frac{d\vec{r}_i}{dt}=\vec{v}_i,\qquad \frac{d\vec{v}_i}{dt}=\vec{a}_T-\vec{a}_{M_i},
$$

donde $\vec{a}_T$ corresponde a la aceleración del objetivo y $\vec{a}_{M_i}$ a la aceleración del $i$-ésimo proyectil. De la formulación general, tenemos que esta dinámica se explicita como

$$
\begin{align}\frac{d\vec{r}_i}{dt}&= \vec{v}_i, \qquad \qquad \qquad \qquad \qquad \quad \ \ \vec{r}_i(0)=\vec{x}_{T,0}-\vec{x}_{M_i,0},\quad \forall i \in I\\

\frac{d\vec{v}_i}{dt}&= a_{T}J\hat{v}_T-u_i\langle \hat{r}_i,\hat{v}_{M_i}  \rangle J\hat{v}_{M_i} , \qquad \vec{v}_i(0)=\vec{v}_{T,0}-\vec{v}_{M_i,0},\quad \  \forall i \in I\\

\frac{d\vec{x}_T}{dt}&= \vec{v}_T,\qquad \qquad \qquad \qquad \qquad \quad \ \vec{x}_T(0)=\vec{x}_{T,0},\\

\frac{d\vec{v}_{T}}{dt}&= a_{T}J\hat{v}_T, \qquad \qquad\qquad  \qquad\quad \ \ \vec{v}_{T}(0)=\vec{v}_{T,0}
\end{align}
$$

Aquí, denotamos 

$$
\hat r_i=\frac{\vec r_i}{\lVert\vec r_i\rVert},\qquad
\hat v_{M_i}=\frac{\vec v_T-\vec v_i}{\lVert\vec v_T-\vec v_i\rVert},\qquad
\hat v_T=\frac{\vec v_T}{\lVert\vec v_T\rVert}.
$$

Al igual que antes, como las aceleraciones son perpendiculares a las velocidades, sus magnitudes se mantienen constantes, por lo que las direcciones $\hat v_T$ y $\hat v_{M_i}$ permanecen definidas. Además, la aceleración de cada proyectil está controlada por $u_i$, de modo que el control general está dado por

$$
u=(u_1,\ldots,u_N).
$$

Estas dinámicas salen de asumir que los proyectiles son independientes entre sí pero condicionados a la trayectoria del objetivo, por lo que podemos aplicar el modelamiento general a cada uno de estos. Eso sí, la trayectoria del objetivo sigue siendo una sola.

## Problema de control

Consideramos el sistema obtenido anteriormente, con estado

$$
x=(\vec r_1,\vec v_1,\ldots,\vec r_N,\vec v_N,\vec x_T,\vec v_T)\in\mathbb R^{4N+4},
$$

y control $u=(u_1,\ldots,u_N)$. Supondremos que la aceleración $a_T=a_T(t)$ del objetivo es conocida, de modo que la dinámica se puede escribir como $\dot x=f(t,x,u)$.

### Condición Final

Buscamos determinar un tiempo final común $t_f>0$ y controles $u_i(t)$ que permitan a todos los proyectiles alcanzar al objetivo en dicho instante. Para ello, consideramos una tolerancia $\varepsilon>0$ y definimos $R_i(t)=\lVert\vec r_i(t)\rVert$, de modo que las condiciones terminales quedan dadas por

$$
R_i(t_f)=\lVert\vec x_T(t_f)-\vec x_{M_i}(t_f)\rVert\leq\varepsilon,\qquad \forall i\in I.
$$

Así, definimos el conjunto objetivo como

$$
\mathcal T=\{x\in\mathbb R^{4N+4}:\lVert\vec r_i\rVert^2\leq\varepsilon^2,\quad\forall i\in I\}.
$$

Cada trayectoria se considera en todo el intervalo $[0,t_f]$, incluso si alguno de los proyectiles entra antes en la región de tolerancia. Al tiempo $t_f$, todos deben estar dentro de dicha región, aunque algunos pueden estar en su interior y otros en su frontera. Las velocidades y orientaciones finales quedan libres igual que en el [modelamiento no lineal](/docs/01_modelo_no_lineal.md).

### Problema de Bolza a tiempo final libre

Para formular el problema de control óptimo, consideramos el conjunto compacto de controles

$$
U=\prod_{i\in I}[-u_{\max,i},u_{\max,i}],\qquad u_{\max,i}>0,
$$

donde $u_{\max,i}$ es la cota de la magnitud del control del $i$-ésimo proyectil. Así, para cada tiempo final $t_f>0$, consideramos controles $u\in L^\infty([0,t_f],U)$.

Para la función de costo se obtiene de manera análoga al modelamiento no lineal, donde consideraremos una función $\ell$ que penaliza la magnitud de los controles y el tiempo común para llegar al objetivo, mediante la expresión

$$
\ell(x,u)=\frac{\alpha}{2}\sum_{i\in I}u_i^2+\beta,\qquad \alpha>0,\quad\beta>0,
$$

en donde $\alpha$ y $\beta$ controlan qué tanto se penaliza el esfuerzo total de control y el tiempo de llegada respectivamente. En este caso, la penalización del tiempo se aplica una sola vez, pues todos los proyectiles comparten el mismo $t_f$. Como la distancia final está restringida por el conjunto objetivo y dejaremos libres las velocidades y orientaciones finales, tomamos el costo terminal $g(t_f,x(t_f))=0$.

De esta forma, el problema de control óptimo está dado por

$$
\min_{\substack{t_f>0\\u\in L^\infty([0,t_f],U)}}
J(u,t_f):=\int_0^{t_f}\ell(x(t),u(t))\,dt+g(t_f,x(t_f))
=\frac{\alpha}{2}\sum_{i\in I}\int_0^{t_f} u_i(t)^2\,dt+\beta t_f,
$$

sujeto a la dinámica $\dot x=f(t,x,u)$ y condiciones iniciales obtenidas anteriormente, y las condiciones terminales

$$
h_i(x(t_f))\leq0,\qquad h_i(x):=\lVert\vec r_i\rVert^2-\varepsilon^2,\qquad\forall i\in I.
$$

Así, obtenemos un problema de Bolza a tiempo final libre y costo terminal nulo, sujeto a $e[x]:=(t_f,x(t_f))\in B$, con

$$
B=\{(t_f,x)\in(0,+\infty)\times\mathbb R^{4N+4}:h_i(x)\leq0,\quad\forall i\in I\}.
$$

Con este planteamiento, aunque las dinámicas de los proyectiles son independientes entre sí salvo por la trayectoria del objetivo, se obliga a que los proyectiles lleguen al objetivo en tiempo $t_f$. Eso sí, no necesariamente pueden llegar todos por primera vez al objetivo a tiempo $t_f$, pues nada restringe que un proyectil se quede cerca del objetivo hasta que el resto de proyectiles lo alcancen también.

Para favorecer que los proyectiles lleguen por primera vez en tiempos cercanos, introducimos un vector de variables de holgura $s=(s_1,\ldots,s_N)$ acotadas por $0\leq s_i(t)\leq \varepsilon^2$ para todo $i\in I$, y ampliamos el vector de control a $w=(u,s)\in L^\infty([0,t_f],W)$, donde

$$
W = U \times [0,\varepsilon^2]^N.
$$

Las variables de holgura relajan la restricción de permanencia fuera de la zona de tolerancia mediante la restricción

$$
R_i(t)^2 + s_i(t) \geq \varepsilon^2,\qquad \forall t\in[0,t_f],\quad \forall i\in I.
$$

De este modo, mientras el $i$-ésimo proyectil se encuentre fuera o en la frontera de la región objetivo ($R_i(t)\geq \varepsilon$), la restricción se satisface con $s_i(t)=0$, pero si entra antes ($R_i(t)<\varepsilon$), la desigualdad obliga a que $s_i(t)\geq \varepsilon^2 - R_i(t)^2 > 0$. Con esto, redefinimos $\ell$ como

$$
\ell(x,w)=\beta+\sum_{i\in I}\left(\frac{\alpha}{2}u_i^2+\gamma s_i\right),\qquad \alpha>0,\ \beta>0,\ \gamma>0,
$$

donde $\gamma$ actúa como un parámetro que penaliza la llegada temprana de los proyectiles al objetivo.

## Caso particular con objetivo estático

Para este caso, asumimos que el objetivo permanece fijo en $\vec x_{T,0}$, de modo que $\vec v_T=\vec a_T=0$. Así, para cada $i\in I$, tenemos que $\vec r_i=\vec x_{T,0}-\vec x_{M_i}$ y $\vec v_i=-\vec v_{M_i}$. La rapidez $V_{M_i}=\lVert\vec v_{M_i,0}\rVert>0$ de cada proyectil es constante, por lo que podemos escribir

$$
\vec r_i=(r_{x_i},r_{y_i}),\qquad
\vec v_{M_i}=V_{M_i}(\cos\theta_{M_i},\sin\theta_{M_i}).
$$

De esta forma, basta con considerar la posición relativa y la orientación de cada proyectil. El estado completo queda dado por

$$
x=(r_{x_1},r_{y_1},\theta_{M_1},\ldots,r_{x_N},r_{y_N},\theta_{M_N})\in\mathbb R^{3N},
$$

y el control ampliado con variables de holgura por $w=(u,s)\in L^\infty([0,t_f],W)$, con $u=(u_1,\ldots,u_N)\in U$, $s=(s_1,\ldots,s_N)\in [0,\varepsilon^2]^N$ y $W=U\times [0,\varepsilon^2]^N$. La posición del objetivo queda como un dato fijo.

Para obtener las dinámicas de los proyectiles, aplicamos el mismo razonamiento que en el [modelamiento no lineal](/docs/02_modelo_no_lineal.md), salvo que ahora los controles $u_i$ son perpendiculares a la velocidad de movimiento en lugar de la LOS, para mayor estabilidad numérica. Con eso, obtenemos el sistema

$$
\begin{align}
\dot r_{x_i}&=-V_{M_i}\cos\theta_{M_i},\\
\dot r_{y_i}&=-V_{M_i}\sin\theta_{M_i},\\
\dot\theta_{M_i}&=\frac{u_i}{V_{M_i}},
\qquad \forall i\in I,
\end{align}
$$

el cual puede ser escrito de la forma $\dot{x}=f(x,u)$. Las condiciones iniciales se obtienen de las posiciones y velocidades iniciales mediante

$$
\begin{align}
r_{x_i}(0)&=x_{T,0}^{(1)}-x_{M_i,0}^{(1)},\\
r_{y_i}(0)&=x_{T,0}^{(2)}-x_{M_i,0}^{(2)},\\
\theta_{M_i}(0)&=\theta_{M_i,0},
\qquad \qquad\forall i\in I.
\end{align}
$$

Por otro lado, el conjunto objetivo queda dado por

$$
\mathcal T=\{x\in\mathbb R^{3N}:r_{x_i}^2+r_{y_i}^2\leq\varepsilon^2,\quad\forall i\in I\}.
$$

Por lo tanto, las condiciones terminales son

$$
h_i(x(t_f))\leq0,\qquad
h_i(x):=r_{x_i}^2+r_{y_i}^2-\varepsilon^2,\qquad\forall i\in I,
$$

y las orientaciones finales $\theta_{M_i}(t_f)$ quedan libres. Al igual que en el modelamiento multiproyectil general, para penalizar la entrada anticipada en la región de tolerancia antes de $t_f$, incorporamos las restricciones de trayectoria con holgura

$$
r_{x_i}(t)^2+r_{y_i}(t)^2+s_i(t)\geq\varepsilon^2,\qquad \forall t\in[0,t_f],\quad\forall i\in I.
$$

Por último, la función de costo instantáneo queda dada por

$$
\ell(x,w)=\beta+\sum_{i\in I}\left(\frac{\alpha}{2}u_i^2+\gamma s_i\right),
\qquad \alpha>0,\quad\beta>0,\quad\gamma>0.
$$

Con costo terminal $g(t_f,x(t_f))=0$, el problema de control óptimo consiste en

$$
\min_{\substack{t_f>0\\w=(u,s)\in L^\infty([0,t_f],W)}}\quad
J(w,t_f):=\int_0^{t_f}\left(\beta+\frac{\alpha}{2}\sum_{i\in I}u_i(t)^2+\gamma\sum_{i\in I}s_i(t)\right)\,dt
$$

sujeto a la dinámica no lineal $\dot x=f(x,u)$, la condición inicial $x(0)=x_0$, las condiciones terminales $h_i(x(t_f))\leq0$ y las restricciones de trayectoria $r_{x_i}(t)^2+r_{y_i}(t)^2+s_i(t)\geq\varepsilon^2$ para todo $i\in I$.