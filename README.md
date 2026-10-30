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

Si todavía no están instaladas las dependencias, ejecutar una vez `Pkg.instantiate()` desde Julia, iniciada en la carpeta del repositorio con `julia --project=.` (ver Instalación).

## Documentación

Para detalles sobre el modelamiento, ver `docs/`.

# Planeación

- [x] Obtener modelamiento del problema general.
- [x] Programar el caso lineal con un proyectil y target.
- [x] Programar el caso no lineal con un proyectil y target.
- [ ] Estudiar la controlabilidad, observabilidad y estabilidad de los problemas.
- [ ] Realizar simulaciones, comparando distintos métodos numéricos.
- [ ] Programar el caso con 2 proyectiles.
- [ ] Programar el caso con $N$ proyectiles.
