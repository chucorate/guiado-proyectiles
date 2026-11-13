## Extensión general a múltiples proyectiles

La extensión está basada en el [planteamiento general](/docs/modelamiento.md). Consideremos ahora un sistema compuesto por $N$ proyectiles $M_i$, con $i\in I:=\{1,\ldots,N\}$, y un único objetivo $T$. Supondremos que el objetivo sigue una trayectoria independiente de los proyectiles y que estos no interactúan directamente entre sí.

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

Buscamos determinar un tiempo final común $T>0$ y controles $u_i(t)$ que permitan a todos los proyectiles alcanzar al objetivo en dicho instante. Para ello, consideramos una tolerancia $\varepsilon>0$ y definimos $R_i(t)=\lVert\vec r_i(t)\rVert$, de modo que las condiciones terminales quedan dadas por

$$
R_i(T)=\lVert\vec x_T(T)-\vec x_{M_i}(T)\rVert\leq\varepsilon,\qquad \forall i\in I.
$$

Así, definimos el conjunto objetivo como

$$
\mathcal T=\{x\in\mathbb R^{4N+4}:\lVert\vec r_i\rVert^2\leq\varepsilon^2,\quad\forall i\in I\}.
$$

Cada trayectoria se considera en todo el intervalo $[0,T]$, incluso si alguno de los proyectiles entra antes en la región de tolerancia. Al tiempo $T$, todos deben estar dentro de dicha región, aunque algunos pueden estar en su interior y otros en su frontera. Las velocidades y orientaciones finales quedan libres igual que en el [modelamiento no lineal](/docs/modelo_no_lineal.md).

### Problema de Bolza a tiempo final libre

Para formular el problema de control óptimo, consideramos el conjunto compacto de controles

$$
U=\prod_{i\in I}[-u_{\max,i},u_{\max,i}],\qquad u_{\max,i}>0,
$$

donde $u_{\max,i}$ es la cota de la magnitud del control del $i$-ésimo proyectil. Así, para cada tiempo final $T>0$, consideramos controles $u\in L^\infty([0,T],U)$

Para la función de costo se obtiene de manera análoga al modelamiento no lineal, donde consideraremos una función $\ell$ que penaliza la magnitud de los controles y el tiempo común para llegar al objetivo, mediante la expresión

$$
\ell(x,u)=\frac{\alpha}{2}\sum_{i\in I}u_i^2+\beta,\qquad \alpha>0,\quad\beta>0,
$$

en donde $\alpha$ y $\beta$ controlan qué tanto se penaliza el esfuerzo total de control y el tiempo de llegada respectivamente. En este caso, la penalización del tiempo se aplica una sola vez, pues todos los proyectiles comparten el mismo $T$. Como la distancia final está restringida por el conjunto objetivo y dejaremos libres las velocidades y orientaciones finales, tomamos el costo terminal $g(T,x(T))=0$.

De esta forma, el problema de control óptimo está dado por

$$
\min_{\substack{T>0\\u\in L^\infty([0,T],U)}}
J(u,T):=\int_0^T\ell(x(t),u(t))\,dt+g(T,x(T))
=\frac{\alpha}{2}\sum_{i\in I}\int_0^T u_i(t)^2\,dt+\beta T,
$$

sujeto a la dinámica $\dot x=f(t,x,u)$ y condiciones iniciales obtenidas anteriormente, y las condiciones terminales

$$
h_i(x(T))\leq0,\qquad h_i(x):=\lVert\vec r_i\rVert^2-\varepsilon^2,\qquad\forall i\in I.
$$

Así, obtenemos un problema de Bolza a tiempo final libre y costo terminal nulo, sujeto a $e[x]:=(T,x(T))\in B$, con

$$
B=\{(T,x)\in(0,+\infty)\times\mathbb R^{4N+4}:h_i(x)\leq0,\quad\forall i\in I\}.
$$

Con este planteamiento, aunque las dinámicas de los proyectiles son independientes entre sí salvo por la trayectoria del objetivo, se obliga a que los proyectiles lleguen al objetivo en tiempo $T$. Eso sí, no necesariamente pueden llegar todos por primera vez al objetivo a tiempo $T$, pues nada restringe que un proyectil se quede cerca del objetivo hasta que el resto de proyectiles lo alcancen también.
