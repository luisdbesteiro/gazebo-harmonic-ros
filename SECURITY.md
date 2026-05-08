# Publicacion segura

Este repositorio esta preparado para publicar solo los archivos necesarios para reproducir la simulacion dentro de Docker.

## No publicar

- Archivos `.env`, claves privadas, certificados, tokens o credenciales.
- `gazebo_config/`: contiene estado local, cache de Gazebo/Fuel y logs. Los logs pueden incluir URLs firmadas temporales.
- `workspace/ros2_gz_ws/build/`, `workspace/ros2_gz_ws/install/` y `workspace/ros2_gz_ws/log/`.
- Bolsas ROS, bases de datos SQLite, videos y resultados brutos pesados salvo que esten anonimizados y sean intencionales.

## Antes de subir

Ejecuta desde este directorio:

```bash
git ls-files --cached --others --exclude-standard -z | xargs -0 rg -n -i "(api[_-]?key|secret|token|password|passwd|private[_-]?key|authorization|bearer|credential|client[_-]?secret|BEGIN (RSA|OPENSSH|PRIVATE)|ghp_|github_pat_|sk-[A-Za-z0-9])"
git status --ignored --short
```

Si el primer comando encuentra algo que no sea documentacion o un falso positivo revisado, no lo subas.

## Resultados

Usa `workspace/results/` para resultados curados y pequenos: graficas, tablas, resumenes Markdown o configuraciones exactas de ejecucion.
Guarda datos brutos en `workspace/results/raw/`; esta carpeta esta excluida por defecto.
