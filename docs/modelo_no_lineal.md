
## Caso No Lineal

Para el caso no lineal, asumimos que el objetivo está fijo, es decir, que $\vec v_T=\vec a_T=0$. En dicho caso, se tiene que $\vec x_T(t)=\vec x_{T,0}$ para todo $t\geq 0$, y del sistema general se deduce que

$$
\frac{d\vec r}{dt}=\vec v,\quad \frac{d\vec v}{dt}=-u\langle\hat r,\hat v_M\rangle J\hat v_M,\quad \frac{d\vec v_T}{dt}=0.
$$

Como $\vec v=\vec v_T-\vec v_M$, y dado que $\vec v_T=0$, tenemos que $\vec v=-\vec v_M$.

Escribiendo $\vec{r}=(r_{x},r_{y})$ y $\vec{v}=-V_{M}(\cos\theta_{M},\sin\theta_{M})$, tenemos que 

$$
\frac{d\vec{r}}{dt}=\vec{v}\implies(\dot{r}_{x},\dot{r}_{y})=-V_{M}(\cos\theta_{M},\sin\theta_{M})
$$

y 

$$
\frac{d\vec{v}}{dt}=-V_{M}\dot{\theta}_{M}J\hat{v}_{M}\implies V_{M}\dot{\theta}_{M}=u \langle \hat{r}, \hat{v}_{M} \rangle \implies \dot{\theta}_{M}=\frac{u}{V_{M}}\frac{r_{x}\cos\theta_{M}+r_{y}\sin\theta_{M}}{\sqrt{ r^{2}_{x}+r^{2}_{y} }},
$$

de modo que obtenemos el sistema

$$
\begin{align}
\dot{r}_{x}&= -V_M\cos\theta_{M}, \\
\dot{r}_{y}&= -V_{M}\sin\theta_{M}, \\
\dot{\theta}_{M}&= \frac{u}{V_{M}}\frac{r_{x}\cos\theta_{M}+r_{y}\sin\theta_{M}}{\sqrt{ r^{2}_{x}+r^{2}_{y} }},
\end{align}
$$

con condiciones iniciales 

$$
r(0)=x_{T,0}-x_{M,0},\quad \theta_{M}(0)=\text{atan2}(v_{M,0}^{(2)},v_{M,0}^{(1)}),
$$

donde la función $\text{atan2}$ obtiene el ángulo entre las componentes de la velocidad inicial de $M$.

### Modelamiento en función de $R$ y $L$

Recordando que $R=\lVert \vec{r}(t) \rVert$ y $L=\theta_{M}-\lambda$, en el modelamiento en función de ángulos habíamos deducido que

$$
\frac{d}{dt}\vec{r}(t)=\dot{R}\hat{r}+R\dot{\lambda}J\hat{r}.
$$

Como ahora $\frac{d}{dt}\vec{r}(t)=-\vec{v}_{M}$, tenemos que 

$$
\dot{R}\hat{r}+R\dot{\lambda}J\hat{r}=-V_{M}\hat{v}_{M}\implies \dot{R}=-V_{M}\langle \hat{v}_{M}, \hat{r} \rangle =-V_{M}\cos(\theta_{M}-\lambda)=-V_{M}\cos(L).
$$

Al proyectar por $J\hat{r}$ en vez de $\hat{r}$, tenemos la expresión 

$$
R\dot{\lambda}=-V_{M}\langle \hat{v}_{M}, J\hat{r} \rangle =-V_{M}\sin(\theta_{M}-\lambda)\implies\dot{\lambda}=-\frac{V_{M}}{R}\sin(L).
$$

Por otro lado, como $\vec{v}=-\vec{v}_{M}$, tenemos que

$$
\frac{d}{dt}\vec{v}_{M}=-\frac{d}{dt}\vec{v}=u\cos(L)J\hat{v}_{M},
$$

donde además 

$$
\frac{d}{dt}\vec{v}_{M}(t)=\frac{d}{dt}\left( V_{M}\begin{pmatrix}\cos\theta_{M} \\  \sin\theta_{M}\end{pmatrix} \right)=V_{M}\dot{\theta}_{M}J\hat{v}_{M}
$$

pues $V_{M}$ es constante. Así, tenemos que 

$$
u\cos(L)J\hat{v}_{M}=V_{M}\dot{\theta}_{M}J\hat{v}_{M}\implies\dot{\theta}_{M}=\frac{u\cos(L)}{V_{M}}.
$$

Juntando las expresiones de $\dot{\theta}_{M}$ y $\dot{\lambda}$, se concluye que 

$$
\dot{L}=\dot{\theta}_{M}-\dot{\lambda}=\frac{u}{V_{M}}\cos(L)+\frac{V_{M}}{R}\sin(L).
$$

Con esto, podemos describir el movimiento del proyectil en función de su distancia $R$ al objetivo, y su ángulo $L$ relativo a la LOS, usando las ecuaciones

$$
\begin{align}\dot{R}&= -V_{M}\cos(L),\qquad \qquad \qquad \qquad R(0)=R_{0} , \\  \dot{L}&= \frac{u}{V_{M}}\cos(L)+\frac{V_{M}}{R}\sin(L), \qquad  \ \ \ L(0)=L_{0}.\end{align}
$$

### Condición Final

Dado que el objetivo permanece fijo en $\vec x_{T,0}$, buscamos determinar un control $u(t)$ que permita al proyectil alcanzar dicha posición en un tiempo final $t_f>0$, por lo que debemos imponer la condición final $\vec x_M(t_f)=\vec x_{T,0}.$ En términos de la posición relativa, esto es $\vec{r}(t_f) = 0$, lo que equivale a

$$
R(t_f)=0.
$$

Cuando $R=0$ la dinámica está indefinida, por lo que el sistema se puede definir para $t\in[0,t_f)$, y la condición terminal debe interpretarse como

$$
\lim_{t\to t_f^-}\vec r(t)=0.
$$

Para evitar esta singularidad al resolver el problema numéricamente, consideraremos una tolerancia $\varepsilon>0$, de modo que el proyectil alcanza al objetivo cuando $R\leq\varepsilon$. Así, definimos el conjunto objetivo como

$$
\mathcal T=\{x=(r_x,r_y,\theta_M)\in\mathbb R^3:r_x^2+r_y^2\leq\varepsilon^2\}.
$$


De esta forma, podemos estudiar la intercepción mediante la condición terminal $R(t_f)=\varepsilon$, suponiendo $R(0)>\varepsilon$.

### Problema de Bolza a tiempo final libre

Consideramos el sistema no lineal obtenido anteriormente, con estado $x=(r_x,r_y,\theta_M)$ y dinámica $\dot x=f(x,u)$. Para formular el problema de control óptimo, consideramos un conjunto compacto de controles $U=[-u_{\max},u_{\max}]$, con $u_{\max}>0$ la cota de la magnitud de la aceleración del proyectil.

Para cada tiempo final $T>0$, consideramos controles $u\in L^\infty([0,T],U)$ cuya trayectoria existe en $[0,T]$. Las condiciones iniciales se obtienen de las posiciones y la velocidad inicial del proyectil mediante

$$
x(0)=x_0=\left(x_{T,0}^{(1)}-x_{M,0}^{(1)},\ x_{T,0}^{(2)}-x_{M,0}^{(2)},\ \operatorname{atan2}(v_{M,0}^{(2)},v_{M,0}^{(1)})\right).
$$

Para construir la función de costo, consideraremos una función $\ell$ que penaliza la magnitud del control y el tiempo para llegar al objetivo, mediante la expresión

$$
\ell(x,u)=\frac{\alpha}{2}u^2+\beta,\qquad \alpha>0,\quad\beta>0,
$$

en donde $\alpha$ y $\beta$ controlan qué tanto se penaliza el esfuerzo de control y el tiempo de llegada respectivamente. Como no penalizaremos la posición cerca del objetivo ni la orientación final del proyectil, tomamos el costo terminal $g(T,x(T))=0$. De esta forma, el problema de control óptimo consiste en

$$
\min_{\substack{T>0\\u\in L^\infty([0,T],U)}}
J(u,T):=\int_0^T\ell(x(t),u(t))\,dt+g(T,x(T)),
$$

sujeto a la dinámica no lineal, la condición inicial $x(0)=x_0$ y la condición terminal

$$
h(x(T))=0,\qquad h(x):=r_x^2+r_y^2-\varepsilon^2,
$$

de modo que la dinámica enstá en una región donde $R>0$ y no aparece la singularidad en $R=0$. En la práctica agregaremos un pequeño delta $\delta>0$ al denominador para que las simulaciones numéricas sean más estables.

Así, obtenemos un problema de Bolza a tiempo final libre y costo terminal nulo, que en particular también es un problema de Lagrange. Se tiene además $e[x]:=(T,x(T))\in B$, con

$$
B=\{(T,x)\in[0,+\infty)\times\mathbb R^3:h(x)=0\}.
$$


## Principio de Pontryagin

Según el principio de Pontryagin para problemas de Bolza, si $(x(\cdot),u(\cdot),T)$ es una solución óptima, existen un multiplicador $p_0\geq0$ y un coestado $p=(p_x,p_y,p_\theta)$ absolutamente continuo, con $(p_0,p(\cdot))$ no trivial. Para simplificar la explicación, trabajaremos primero con el caso normal, es decir, suponiendo $p_0>0$. En este caso podemos dividir todos los multiplicadores por $p_0$ y tomar $p_0=1$, de modo que el prehamiltoniano queda dado por

$$
H(x,u,p):=\ell(x,u)+\langle p,f(x,u)\rangle.
$$

Para simplificar la escritura, denotamos

$$
R(x)=\sqrt{r_x^2+r_y^2},\qquad c(x)=\frac{r_x\cos\theta_M+r_y\sin\theta_M}{R(x)}.
$$

Al reemplazar la dinámica y la función de costo, obtenemos

$$
H=\frac{\alpha}{2}u^2+\beta-V_M(p_x\cos\theta_M+p_y\sin\theta_M)+\frac{p_\theta u}{V_M}c(x).
$$

El estado y el coestado deben cumplir $\dot x=\partial_pH$ y $\dot p=-\partial_xH$. La primera ecuación recupera la dinámica original. Por otro lado, tenemos que

$$
\begin{align}
\frac{\partial c}{\partial r_x}&=\frac{\cos\theta_M}{R}-\frac{r_xc}{R^2},\\
\frac{\partial c}{\partial r_y}&=\frac{\sin\theta_M}{R}-\frac{r_yc}{R^2},\\
\frac{\partial c}{\partial\theta_M}&=\frac{-r_x\sin\theta_M+r_y\cos\theta_M}{R}.
\end{align}
$$

De esta forma, al derivar el Hamiltoniano respecto del estado, se obtiene

$$
\begin{align}
\dot p_x&=-\frac{p_\theta u}{V_M}\left(\frac{\cos\theta_M}{R}-\frac{r_xc}{R^2}\right),\\
\dot p_y&=-\frac{p_\theta u}{V_M}\left(\frac{\sin\theta_M}{R}-\frac{r_yc}{R^2}\right),\\
\dot p_\theta&=V_M(p_y\cos\theta_M-p_x\sin\theta_M)+\frac{p_\theta u}{V_MR}(r_x\sin\theta_M-r_y\cos\theta_M).
\end{align}
$$

Por otro lado, el control debe satisfacer, para casi todo $t\in[0,T]$,

$$
H(x(t),u(t),p(t))=\min_{w\in U}H(x(t),w,p(t)).
$$

Como $\alpha>0$, el prehamiltoniano es estrictamente convexo respecto del control. Su mínimo sin restricciones cumple

$$
\frac{\partial H}{\partial u}=\alpha u+\frac{p_\theta}{V_M}c(x)=0\implies u=-\frac{p_\theta c(x)}{\alpha V_M}.
$$

Al considerar la restricción $u\in[-u_{\max},u_{\max}]$, se concluye que

$$
u(t)=\operatorname{proj}_{[-u_{\max},u_{\max}]}\left(-\frac{p_\theta(t)c(x(t))}{\alpha V_M}\right).
$$

### Condiciones necesarias en $T$

Por otro lado y denotando $\Theta(t):=H(x(t),u(t),p(t))$ con $u$ óptimo y $p_0=1$, para un problema de Bolza con tiempo y estado iniciales fijos, la condición general de transversalidad se escribe como

$$
p_{0}dg+(\Theta(t)dt-\langle p, dx_{t} \rangle)|_{t=t_{0}}^{t=T}=0,\quad (dt_{0},dx_{0},dT,dx_{f})\in T_{e[x]}B
$$

Como $t_{0},x_{0}$ son fijos, y además $g \equiv 0$, la condición se reduce a

$$
\Theta(T)dT-\langle p(T),dx  \rangle=0,\quad (dT,dx) \in T_{e[x]}B.
$$

Como $B$ está definido por $h(x_f)=0$ sin dependencia explícita del tiempo, las variaciones cumplen $\langle\nabla h(x(T)),d x\rangle=0$, mientras que $dT$ es libre.
  
Tomando $dx=0$ y variando $d T$, se obtiene $\Theta(T)=0$. Por otro lado, al tomar $d T=0$, se deduce que $p(T)$ es perpendicular a todas las variaciones tangentes a la frontera del objetivo. Además, $\nabla h(x(T))=(2r_x(T),2r_y(T),0)\neq0$ para $\varepsilon>0$.

Como el espacio normal a la frontera está generado por $\nabla h(x(T))$ y tiene dimensión 1 (dado que estamos en $\mathbb{R}^3$ y el espacio tangente es de dimensión 2 al tener una restricción $\langle \nabla h(x(T)), dx\rangle=0$ con $\nabla h(x(T))\neq0$), existe $\nu\in\mathbb R$ tal que $p(T)=\nu\nabla h(x(T))$. Así, llegamos a

$$
\begin{align}
p_x(T)&=2\nu r_x(T),\\
p_y(T)&=2\nu r_y(T),\\
p_\theta(T)&=0,\\
\Theta(T)&=0.
\end{align}
$$

Usando $p_\theta(T)=0$ en la expresión del control, tenemos que 

$$
u(T)=\operatorname{proj}_{[-u_{\max},u_{\max}]}\left(-\frac{p_\theta(T)c(x(T))}{\alpha V_M}\right)
=0,
$$

y reemplazando luego las condiciones de transversalidad en el prehamiltoniano, obtenemos finalmente la siguiente condición:

$$
\begin{align}
0&=\Theta(T)=H(x(T),u(T),p(T))\\
&=\frac{\alpha}{2}u(T)^2+\beta-V_M\left(p_x(T)\cos\theta_M(T)+p_y(T)\sin\theta_M(T)\right)
+\frac{p_\theta(T)u(T)}{V_M}c(x(T))\\
&=\beta-V_M\left(p_x(T)\cos\theta_M(T)+p_y(T)\sin\theta_M(T)\right)\\
&=\beta-V_M\left(2\nu r_x(T)\cos\theta_M(T)+2\nu r_y(T)\sin\theta_M(T)\right)\\
&=\beta-2\nu V_M\left(r_x(T)\cos\theta_M(T)+r_y(T)\sin\theta_M(T)\right)\\
&=\beta-2\nu V_M \langle r(T), \hat{v}_{M}(T) \rangle.
\end{align}
$$

Así,

$$
\nu \langle \vec{r}(T), \vec{v}_M(T)\rangle = \frac{\beta}{2}.
$$

### Caso particular

En general, obtener una solución analítica es difícil y por lo tanto el problema debe resolverse con métodos numéricos. En lo que sigue se dará un ejemplo de solución particular cuando el proyectil comienza apuntando al objetivo, es decir cuando $\theta_M(0) = 0$ (pues estamos asumiendo que la LOS es paralela al eje x).

Tomamos la LOS inicial sobre el semieje positivo de $x$, de modo que las condiciones iniciales son $x_0=(R_0,0,0)$, con $R_0>\varepsilon$. Buscamos una trayectoria rectilínea que mantenga $r_y(t)=0$ y $\theta_M(t)=0$. En este caso, $c(x(t))=1$ mientras $r_x(t)>0$, por lo que la ecuación de $\theta_M$ exige

$$
\dot\theta_M(t)=\frac{u(t)}{V_M}=0\implies u(t)=0.
$$

Al reemplazar este control en las ecuaciones del estado, obtenemos $\dot r_x=-V_M$ y $\dot r_y=0$. Así, la trayectoria queda dada por

$$
x(t)=(R_0-V_Mt,0,0).
$$

Como buscamos la primera llegada a la frontera del objetivo, se tiene $r_x(T)=\varepsilon$, de modo que

$$
R_0-V_MT=\varepsilon\implies T^*=\frac{R_0-\varepsilon}{V_M}.
$$

Por último, para cualquier control admisible que alcance el objetivo se tiene

$$
J(u^*,T)=\frac{\alpha}{2}\int_0^Tu^*(t)^2\,dt+\beta T = \beta T \geq \beta T^* =J(u^*,T^*)\implies J^* = \beta T^*.
$$

Por lo tanto, para este caso particular obtenemos la solución óptima

$$
\begin{align}
u^*(t)&=0,\\
x^*(t)&=(R_0-V_Mt,0,0),\qquad 0\leq t\leq T^*,\\
T^*&=\frac{R_0-\varepsilon}{V_M},\\
J^*&=\frac{\beta(R_0-\varepsilon)}{V_M}.
\end{align}
$$
