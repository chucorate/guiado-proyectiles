# Control Óptimo en el Guiado de Proyectiles

## Instalación

### Requisitos previos

- Julia 1.12+.

### Pasos de instalación

1. Clonar el repositorio y abrir una terminal en la carpeta que contiene `Project.toml`.
2. Iniciar Julia en la terminal:

   ```sh
   julia --project=.
   ```

3. En la consola de Julia, instalar todas las dependencias del proyecto:

   ```julia
   using Pkg
   Pkg.instantiate()
   ```

## Uso

Los notebooks de `src/modelos/` usan el kernel de Julia de Jupyter. La primera celda activa automáticamente el entorno de este repositorio.

Los gráficos y GIF generados se guardan en `outputs/`, organizados en subcarpetas según el modelo.

Si todavía no están instaladas las dependencias, ejecutar una vez `Pkg.instantiate()` desde Julia, iniciada en la carpeta del repositorio con `julia --project=.` (ver Instalación).

## Documentación

Para detalles sobre el modelamiento, ver `docs/`.

# Planeación

- [x] Obtener modelamiento del problema general.
- [x] Programar el caso lineal con un proyectil y target.
- [x] Programar el caso no lineal con un proyectil y target.
- [x] Estudiar la controlabilidad, observabilidad y estabilidad de los problemas.
- [ ] Realizar simulaciones, comparando distintos métodos numéricos.
- [x] Estudiar la aplicabilidad de HJB.
- [x] Programar el caso con 2 proyectiles.
- [x] Programar el caso con $N$ proyectiles.
- [x] Investigar sobre posibles soluciones analíticas.

# Referencias

Arita, S., & Ueno, S. (2013,). *Optimal Feedback Guidance for Nonlinear Missile Model with Impact Time and Angle Constraints*. American Institute of Aeronautics, Astronautics. [https://doi.org/10.2514/6.2013-4785](https://doi.org/10.2514/6.2013-4785)

Jeon, I.-S., Lee, J.-I., & Tahk, M.-J. (2006). Impact-time-control guidance law for anti-ship missiles. *IEEE Transactions on Control Systems Technology*, *14*(2), 260–266. [https://doi.org/10.1109/TCST.2005.863655](https://doi.org/10.1109/TCST.2005.863655)

Lee, D., Kim, J.-H., & Choi, H.-L. (2016,). *CLOSED-FORM COOPERATIVE GUIDANCE LAW FOR TWO MISSILES WITH COUPLED TERMINAL VELOCITY CONSTRAINTS*. [https://api.semanticscholar.org/CorpusID:189808340](https://api.semanticscholar.org/CorpusID:189808340)

Palumbo, N. F., Blauwkamp, R. A., & Lloyd, J. M. (2010). Modern Homing Missile Guidance Theory and Techniques. *Johns Hopkins Apl Technical Digest*, *29*, 42–59. [https://api.semanticscholar.org/CorpusID:124469648](https://api.semanticscholar.org/CorpusID:124469648)

Zarchan, P. (2012). *Tactical and strategic missile guidance / Paul Zarchan, MIT Lincoln Laboratory, Lexington, Massachusetts.* (Sixth edition.). American Institute of Aeronautics, Astronautics.
