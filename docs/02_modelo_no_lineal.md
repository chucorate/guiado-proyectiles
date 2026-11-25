
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

Consideramos el sistema no lineal obtenido anteriormente, con estado $x=(r_x,r_y,\theta_M)$ y dinámica $\dot x=f(x,u)$. Para formular el problema de control óptimo, consideramos un conjunto compacto de controles $U=[-u_{\max},u_{\max}]$, con $u_{\max}>0$ la cota de la magnitud del control.

Para cada tiempo final $t_f>0$, consideramos controles $u\in L^\infty([0,t_f],U)$ cuya trayectoria existe en $[0,t_f]$. Las condiciones iniciales se obtienen de las posiciones y la velocidad inicial del proyectil mediante

$$
x(0)=x_0=\left(x_{T,0}^{(1)}-x_{M,0}^{(1)},\ x_{T,0}^{(2)}-x_{M,0}^{(2)},\ \mathrm{atan2}(v_{M,0}^{(2)},v_{M,0}^{(1)})\right).
$$

Para construir la función de costo, consideraremos una función $\ell$ que penaliza la magnitud del control y el tiempo para llegar al objetivo, mediante la expresión

$$
\ell(x,u)=\frac{\alpha}{2}u^2+\beta,\qquad \alpha>0,\quad\beta>0,
$$

en donde $\alpha$ y $\beta$ controlan qué tanto se penaliza el esfuerzo de control y el tiempo de llegada respectivamente. Como no penalizaremos la posición cerca del objetivo ni la orientación final del proyectil, tomamos el costo terminal $g(t_f,x(t_f))=0$. De esta forma, el problema de control óptimo consiste en

$$
\min_{\substack{t_f>0\\u\in L^\infty([0,t_f],U)}}
J(u,t_f):=\int_0^{t_f}\ell(x(t),u(t))\,dt+g(t_f,x(t_f)),
$$

sujeto a la dinámica no lineal, la condición inicial $x(0)=x_0$ y la condición terminal

$$
h(x(t_f))=0,\qquad h(x):=r_x^2+r_y^2-\varepsilon^2,
$$

de modo que la dinámica está en una región donde $R\geq\varepsilon>0$ y no aparece la singularidad en $R=0$.

Así, obtenemos un problema de Bolza a tiempo final libre y costo terminal nulo, que en particular también es un problema de Lagrange. Se tiene además $e[x]:=(t_f,x(t_f))\in B$, con

$$
B=\{(t_f,x)\in[0,+\infty)\times\mathbb R^3:h(x)=0\}.
$$

## Principio de Pontryagin

Según el principio de Pontryagin para problemas de Bolza, si $(x(\cdot),u(\cdot),t_f)$ es una solución óptima, existen un multiplicador $p_0\geq0$ y un coestado $p=(p_x,p_y,p_\theta)$ absolutamente continuo, con $(p_0,p(\cdot))$ no trivial. Para simplificar la explicación, trabajaremos primero con el caso normal, es decir, suponiendo $p_0>0$. En este caso podemos dividir todos los multiplicadores por $p_0$ y tomar $p_0=1$, de modo que el prehamiltoniano queda dado por

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

Por otro lado, el control debe satisfacer, para casi todo $t\in[0,t_f]$,

$$
H(x(t),u(t),p(t))=\min_{w\in U}H(x(t),w,p(t)).
$$

Como $\alpha>0$, el prehamiltoniano es estrictamente convexo respecto del control. Su mínimo sin restricciones cumple

$$
\frac{\partial H}{\partial u}=\alpha u+\frac{p_\theta}{V_M}c(x)=0\implies u=-\frac{p_\theta c(x)}{\alpha V_M}.
$$

Al considerar la restricción $u\in[-u_{\max},u_{\max}]$, se concluye que

$$
u(t)=\mathrm{proj}_{[-u_{\max},u_{\max}]}\left(-\frac{p_\theta(t)c(x(t))}{\alpha V_M}\right).
$$

### Condiciones necesarias en $t_f$

Por otro lado y denotando $\Theta(t):=H(x(t),u(t),p(t))$ con $u$ óptimo y $p_0=1$, para un problema de Bolza con tiempo y estado iniciales fijos, la condición general de transversalidad se escribe como

$$
p_{0}dg+(\Theta(t)dt-\langle p, dx_{t} \rangle)|_{t=t_{0}}^{t=t_f}=0,\quad (dt_{0},dx_{0},dt_f,dx_{f})\in T_{e[x]}B
$$

Como $t_{0},x_{0}$ son fijos, y además $g \equiv 0$, la condición se reduce a

$$
\Theta(t_f)dt_f-\langle p(t_f),dx  \rangle=0,\quad (dt_f,dx) \in T_{e[x]}B.
$$

Como $B$ está definido por $h(x_f)=0$ sin dependencia explícita del tiempo, las variaciones cumplen $\langle\nabla h(x(t_f)),d x\rangle=0$, mientras que $dt_f$ es libre.
  
Tomando $dx=0$ y variando $dt_f$, se obtiene $\Theta(t_f)=0$. Por otro lado, al tomar $dt_f=0$, se deduce que $p(t_f)$ es perpendicular a todas las variaciones tangentes a la frontera del objetivo. Además, $\nabla h(x(t_f))=(2r_x(t_f),2r_y(t_f),0)\neq0$ para $\varepsilon>0$.

Como el espacio normal a la frontera está generado por $\nabla h(x(t_f))$ y tiene dimensión 1 (dado que estamos en $\mathbb{R}^3$ y el espacio tangente es de dimensión 2 al tener una restricción $\langle \nabla h(x(t_f)), dx\rangle=0$ con $\nabla h(x(t_f))\neq0$), existe $\nu\in\mathbb R$ tal que $p(t_f)=\nu\nabla h(x(t_f))$. Esto significa que $(p_x(t_f),p_y(t_f))$ es perpendicular a la tangente $(-r_y(t_f),r_x(t_f))$ de la circunferencia y que $p_\theta(t_f)=0$ por ser libre la orientación final. Así, llegamos a

$$
\begin{align}
-r_y(t_f)p_x(t_f)+r_x(t_f)p_y(t_f)&=0,\\
p_\theta(t_f)&=0,\\
\Theta(t_f)&=0.
\end{align}
$$

Usando $p_\theta(t_f)=0$ en la expresión del control, tenemos que

$$
u(t_f)=\mathrm{proj}_{[-u_{\max},u_{\max}]}\left(-\frac{p_\theta(t_f)c(x(t_f))}{\alpha V_M}\right)
=0,
$$

y reemplazando luego las condiciones de transversalidad en el prehamiltoniano, obtenemos finalmente la siguiente condición:

$$
\begin{align}
0&=\Theta(t_f)=H(x(t_f),u(t_f),p(t_f))\\
&=\frac{\alpha}{2}u(t_f)^2+\beta\\
&-V_M\left(p_x(t_f)\cos\theta_M(t_f)
+ p_y(t_f)\sin\theta_M(t_f)\right)
+\frac{p_\theta(t_f)u(t_f)}{V_M}c(x(t_f))\\
&=\beta-V_M\left(p_x(t_f)\cos\theta_M(t_f)+p_y(t_f)\sin\theta_M(t_f)\right)\\
&=\beta-\langle p(t_f),\vec{v}_M(t_f)\rangle
\end{align}
$$

de donde se obtiene que $\langle p(t_f),\vec{v}_M(t_f)\rangle=\beta$.

### Caso $p_0=0$

Para este caso, el Hamiltoniano queda dado por

$$
H=-V_M(p_x\cos\theta_M+p_y\sin\theta_M)+\frac{p_\theta u}{V_M}c(x).
$$

Definiendo $\varphi(t)=p_\theta c(x(t))/V_M$, la minimización respecto del control da

$$
u=\begin{cases}
-u_{\max},&\varphi(t)>0,\\
u_{\max},&\varphi(t)<0,\\
\text{cualquier valor de }U,&\varphi(t)=0.
\end{cases}
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

Como buscamos la primera llegada a la frontera del objetivo, se tiene $r_x(t_f)=\varepsilon$, de modo que

$$
R_0-V_Mt_f=\varepsilon\implies t_f^*=\frac{R_0-\varepsilon}{V_M}.
$$

Por último, para cualquier control admisible que alcance el objetivo se tiene

$$
J(u,t_f)=\frac{\alpha}{2}\int_0^{t_f}u(t)^2\,dt+\beta t_f
\geq\beta t_f\geq\frac{\beta(R_0-\varepsilon)}{V_M}.
$$

El control $u^*=0$ alcanza ambas igualdades, pues no utiliza esfuerzo de control y llega siguiendo el segmento más corto al objetivo. Por lo tanto, para este caso particular obtenemos la solución óptima

$$
\begin{align}
u^*(t)&=0,\\
x^*(t)&=(R_0-V_Mt,0,0),\qquad 0\leq t\leq t_f^*,\\
t_f^*&=\frac{R_0-\varepsilon}{V_M},\\
J^*&=\frac{\beta(R_0-\varepsilon)}{V_M}.
\end{align}
$$

## Ecuación de Hamilton-Jacobi-Bellman

Denotemos por $V(x)$ el menor costo de llegada al conjunto $\mathcal T$ desde el estado $x=(r_x,r_y,\theta_M)$. El costo se evalúa sin el término exponencial, de modo que $\lambda=0$ y el término $\lambda V(x)$ se anula. Aquí $\lambda$ denota la tasa de descuento, distinta del ángulo de la línea de mira usado en la dinámica. En los puntos exteriores donde $V$ es finita y diferenciable, se cumple

$$
0=\min_{u\in U}\left\{\ell(x,u)+\langle\nabla V(x),f(x,u)\rangle\right\},
$$

y además $x\in\mathcal T$ se tiene $V(x)=0$. Al sustituir la dinámica cartesiana y el costo $\ell$, obtenemos

$$
0=\beta-V_M\left(\cos\theta_M\,V_{r_x}+\sin\theta_M\,V_{r_y}\right)
+\min_{u\in U}\left\{\frac{\alpha}{2}u^2+\frac{c(x)}{V_M}V_{\theta_M}u\right\},
\qquad R>\varepsilon.
$$

Aquí $c(x)$ es el mismo que se definió para la formulación de Pontryagin. Para evaluar el mínimo, escribimos $\varphi(t)=c(x(t))V_{\theta_M}/V_M$ y completamos el cuadrado,

$$
\frac{\alpha}{2}u^2+\varphi(t)u
=\frac{\alpha}{2}\left(u+\frac{\varphi(t)}{\alpha}\right)^2-\frac{\varphi(t)^2}{2\alpha}.
$$

Si $|\varphi(t)|\leq\alpha u_{\max}$, el cuadrado puede anularse dentro de $U$. Si $|\varphi(t)|>\alpha u_{\max}$, el mínimo se alcanza en un extremo del intervalo. Sustituyendo el valor mínimo en cada caso, la ecuación queda

$$
0=\beta-V_M\left(\cos\theta_M\,V_{r_x}+\sin\theta_M\,V_{r_y}\right)
+\begin{cases}
-\dfrac{\varphi(t)^2}{2\alpha}, & |\varphi(t)|\leq\alpha u_{\max},\\[6pt]
\dfrac{\alpha}{2}u_{\max}^2-u_{\max}|\varphi(t)|, & |\varphi(t)|>\alpha u_{\max}.
\end{cases}
$$

Por lo tanto, como $\alpha>0$, la expresión que minimizamos es estrictamente convexa en el control, por lo que el minimizador es único en $U$ y está dada por

$$
u(t)=\mathrm{proj}_{[-u_{\max},u_{\max}]}\left(-\frac{V_{\theta_M}c(x(t))}{\alpha V_M}\right),
$$

lo cual, junto al hecho que $V(x)=0$ cuando se llega a $\mathcal{T}$, se tiene que la forma del control es equivalente a la entregada por Pontryagin en el caso $p_0=1$, salvo que ahora depende de la función valor en lugar del coestado.
