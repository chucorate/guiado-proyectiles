# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     formats: ipynb,jl:percent
#     text_representation:
#       extension: .jl
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.19.6
#   kernelspec:
#     display_name: Julia 1.12.7
#     language: julia
#     name: julia-1.12
# ---

# %% [markdown]
# # Resultado Analítico

# %% [markdown]
# ## Imports

# %%
import Plots
using LinearAlgebra
import DifferentialEquations as DE
import ControlSystems as CS

# %% [markdown]
# ## Matrices problema linealizado

# %%
A = [
	0 1;
	0 0;
]

B = [
	0,
	-1
] 

# %% [markdown]
# ## Funciones de dinámicas

# %%
function control_clasico!(dx, x, p, t)
	Ap, Bp, u = p
	dx .= Ap*x + Bp*u(t)
end

# %%
function control_feedback!(dx, x, p, t)
	Ap, Bp, Kp = p
	dx .= (Ap+Bp*Kp)*x
end

# %%
function control_paper!(dx, x, p, t)
	Ap, Bp, T_hit = p
	t_go = T_hit - t
	u = 3*(x[1]+x[2]*t_go)/(t_go^2)
	dx .= Ap*x + Bp*u
end

# %%
function control_paper_qf!(dx, x, p, t)
	Ap, Bp, q_f, T_hit = p
	t_go = T_hit - t
	u = 3*(x[1]+x[2]*t_go)/(t_go^2+3/(q_f*t_go))
	dx .= Ap*x + Bp*u
end

# %%
# function riccati!(dE, E, p, t)
# 	Ar, Br, Wr, Ur = p
# 	A, B, W, U = reshape(Ar, 2,2), reshape(Br, 2,2), reshape(Wr, 2,2), reshape(Ur, 2,2) 
# 	Er = reshape(E, 2,2)
# 	dE .= vec(W - A'*Er - Er'*A - Er'*B*B'*Er/U) # <- como u es de dim 1 U es escalar
# end

function riccati!(dE, E, p, t)
	A, B, W, U = p
    Er  = reshape(E, 2, 2)
    dEr = -A' * Er - Er * A - Er * B * (B' * Er) / U + W
    dE .= vec(dEr)
end

# %% [markdown]
# ## Controles

# %%
u1(t) = sin(t)*5

# %% [markdown]
# ## Valores de prueba

# %%
ry0 = -10 # <- al ser la orientación del problema arbitrario podemos fijarla a LOS en t0 para facilitar las condiciones iniciales
vy0 = 20 # [m/s]?
rx0 = 1200 # distancia horizontal
vx0 = -30

T_choque = -rx0/vx0

z0 = [ry0, vy0]

tspan = (0.0, T_choque)

# %% [markdown]
# ## Gráficos

# %% [markdown]
# ### Chequeo funcionamiento

# %%
prob_chk = CS.ODEProblem(control_clasico!, z0, tspan, (A, B, u1))
din_chk = CS.solve(prob_chk)

# %%
Plots.plot(din_chk, labels=["ry" "vy"])

# %% [markdown]
# ### Restricción dura

# %%
prob_paper = CS.ODEProblem(control_paper!, z0, tspan, (A, B, T_choque))
din_paper = CS.solve(prob_paper)

# %%
Plots.plot(din_paper, labels=["ry" "vy"])

# %% [markdown]
# ### Restricciones blandas

# %%
prob_margen = CS.ODEProblem(control_paper_qf!, z0, tspan, (A, B, 1, T_choque))
din_margen = CS.solve(prob_margen)

# %%
Plots.plot(din_margen, labels=["ry" "vy"])

# %% [markdown]
# ### Gráficos trayectorias y choques

# %%
p0 = [rx0, ry0] # centramos los ejes en la posición inicial del misil
B_O = [vx0/2;vy0] # para hacer mas interesante la simulación el aporte de cada objeto en movimiento a la velocidad relativa en x será repartido 
u_O(t) = 1

# %%
prob_obj = CS.ODEProblem(control_clasico!, p0, tspan, (zeros(2,2), B_O, u_O))
din_obj = CS.solve(prob_obj)

# %%
f_x(t) = din_obj(t)[1]
f_y(t) = din_obj(t)[2]

Plots.plot(f_x, f_y, 0, 40, idxs=(1,2))

# %%
g_x(t) = (-vx0/2)*t
g_y(t) = vy0*t-din_paper(t)[1]
Plots.plot(g_x, g_y, 0, 40, idxs=(1,2))


# %%
f_y(40)
