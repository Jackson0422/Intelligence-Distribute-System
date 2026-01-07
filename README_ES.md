# Sistema de Localización Colaborativa Multi-Robot

[English](README.md) | [中文](README_CN.md)

## 📋 Descripción General

Este proyecto implementa un **sistema de localización colaborativa descentralizado** para múltiples robots TurtleBot3 en ROS2. El sistema utiliza Localización Adaptativa de Monte Carlo (AMCL) combinado con un protocolo de consenso basado en gossip para mejorar la precisión de localización a través de la comunicación entre robots.

### Características Principales

- ✅ **Soporte multi-robot**: 2-4 robots TurtleBot3
- ✅ **Arquitectura descentralizada**: No requiere coordinador central
- ✅ **Localización colaborativa**: Los robots comparten información de pose para mejorar la precisión
- ✅ **Planificación de rutas RRT**: Generación de trayectorias sin colisiones
- ✅ **Evaluación en tiempo real**: Seguimiento de errores de posición y orientación
- ✅ **Visualización de datos**: Herramientas completas de gráficos para análisis de rendimiento

### Arquitectura del Sistema

```
┌─────────────┐
│   Gazebo    │ ← Entorno de Simulación
└──────┬──────┘
       │
┌──────┴──────────────────────────────────────┐
│              Red ROS2                        │
├──────────────┬───────────────┬───────────────┤
│   Nodos      │  Localización │  Planificación│
│   AMCL       │  Colaborativa │  de Rutas     │
│              │               │  y Control    │
└──────────────┴───────────────┴───────────────┘
```

## 🛠️ Requisitos

### Dependencias de Software

- **SO**: Ubuntu 22.04 (Jammy)
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **Gazebo**: 11.x

### Paquetes Python

```bash
sudo apt install python3-pip
pip3 install numpy matplotlib pandas
```

### Paquetes ROS2

```bash
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-tf-transformations
```

## 📦 Instalación

### 1. Clonar el Repositorio

```bash
cd ~
git clone <repository-url> ids_roswk
cd ids_roswk
```

### 2. Compilar el Espacio de Trabajo

```bash
cd ~/ids_roswk
colcon build --symlink-install
source install/setup.bash
```

### 3. Configurar Variables de Entorno

Agregar a `~/.bashrc`:

```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ids_roswk/src/localization_evaluation/models
source ~/ids_roswk/install/setup.bash
```

## 🚀 Inicio Rápido

### Ejecutar el Sistema Completo (3 Robots)

Necesitas **5 terminales** para ejecutar el sistema completo de localización colaborativa.

#### Terminal 1: Lanzar Simulación Gazebo

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation multibot_gazebo.launch.py num_robots:=3
```

**Espera hasta que todos los robots se generen en Gazebo antes de continuar.**

#### Terminal 2: Lanzar Localización AMCL

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation amcl_multibot.launch.py num_robots:=3
```

**Espera el mensaje "Managed nodes are active".**

#### Terminal 3: Lanzar Localización Colaborativa

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 launch localization_evaluation decentralized_coloc.launch.py num_robots:=3
```

#### Terminal 4: Lanzar Nodo de Evaluación

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation pose_eval_coloc --ros-args -p num_robots:=3
```

#### Terminal 5: Lanzar Controladores de Robot

```bash
cd ~/ids_roswk
source install/setup.bash
ros2 run localization_evaluation track_multibot --ros-args -p num_robots:=3
```

**Los robots ahora navegarán a través de sus puntos de ruta predefinidos.**

## ⚙️ Configuración

### Número de Robots

El sistema soporta **2, 3 o 4 robots**. Cambia el parámetro `num_robots` en todos los comandos:

```bash
# Para 2 robots
num_robots:=2

# Para 3 robots (predeterminado)
num_robots:=3

# Para 4 robots
num_robots:=4
```

### Posiciones Iniciales de los Robots

Definidas en `multibot_gazebo.launch.py`:

| Robot  | Posición (x, y) | Orientación (yaw) |
|--------|-----------------|-------------------|
| TB3_0  | (-2.0, -0.5)   | 0.0 rad          |
| TB3_1  | (0.0, 0.5)     | 0.0 rad          |
| TB3_2  | (-1.0, -1.5)   | 0.0 rad          |
| TB3_3  | (2.0, 0.0)     | 0.0 rad          |

### Puntos de Ruta

Definidos en `track_multibot.py` (líneas 361-393):

```python
ALL_ROBOTS_CONFIG = {
    'tb3_0': {
        'start': (-2.0, -0.5),
        'waypoints': [(-2.0, -0.5), (-1.0, -0.5), (-1.0, 0.5), (0.0, 2.0)]
    },
    'tb3_1': {
        'start': (0.0, 0.5),
        'waypoints': [(0.0, 0.5), (1.0, 0.5), (1.0, -0.5), (0.0, -2.0)]
    },
    'tb3_2': {
        'start': (-1.0, -1.5),
        'waypoints': [(-1.0, -1.5), (-2.0, -0.5), (-2.0, 0.5), (0.0, 2.0)]
    },
    'tb3_3': {
        'start': (2.0, 0.0),
        'waypoints': [(2.0, 0.0), (2.0, -1.0), (1.0, -1.5), (0.0, -2.0)]
    }
}
```

## 📊 Análisis de Datos

### Registro Automático de Datos

Al ejecutar `pose_eval_coloc`, los datos se guardan automáticamente en:

```
~/ids_roswk/evaluation_results/multibot/coloc/
```

Archivos generados:
- `tb3_X_coloc_eval_TIMESTAMP.csv` - Datos de evaluación sin procesar
- `tb3_X_coloc_statistics_TIMESTAMP.txt` - Estadísticas resumidas

### Visualizar Resultados

Usa el script de procesamiento de datos para generar gráficos:

```bash
cd ~/ids_roswk
python3 src/localization_evaluation/localization_evaluation/data_processing_coloc.py \
    --timestamp 20260107_164136 \
    --num-robots 3
```

**Gráficos generados:**
1. `trajectory_comparison` - Trayectorias reales vs estimadas
2. `position_error_comparison` - Error de posición a lo largo del tiempo
3. `yaw_error_comparison` - Error de orientación a lo largo del tiempo
4. `statistics_comparison` - Gráficos de barras de RMSE, media, errores máximos
5. `error_distribution` - Histogramas de distribuciones de error
6. `xy_error_scatter` - Distribución espacial de errores

**Ubicación de salida:**
```
~/ids_roswk/evaluation_results/multibot/coloc/plots/
```

## 📁 Estructura del Proyecto

```
ids_roswk/
├── src/localization_evaluation/
│   ├── launch/
│   │   ├── multibot_gazebo.launch.py       # Simulación Gazebo
│   │   ├── amcl_multibot.launch.py         # Localización AMCL
│   │   └── decentralized_coloc.launch.py   # Localización colaborativa
│   ├── localization_evaluation/
│   │   ├── track_multibot.py               # Controlador de robot
│   │   ├── pose_eval_coloc.py              # Nodo de evaluación
│   │   ├── decentralized_coloc_agent.py    # Agente de coloc
│   │   ├── pathplan.py                     # Planificador de rutas RRT
│   │   └── data_processing_coloc.py        # Visualización de datos
│   ├── param/
│   │   ├── nav2_params_tb3_0.yaml          # Parámetros AMCL
│   │   ├── nav2_params_tb3_1.yaml
│   │   ├── nav2_params_tb3_2.yaml
│   │   └── nav2_params_tb3_3.yaml
│   ├── models/
│   │   ├── tb3_1/model.sdf                 # Modelos de robot
│   │   ├── tb3_2/model.sdf
│   │   └── tb3_3/model.sdf
│   └── maps/
│       └── map.yaml                        # Mapa del entorno
└── evaluation_results/                     # Datos de salida
```

## 🔬 Detalles Técnicos

### Algoritmo de Localización Colaborativa

El sistema implementa una localización colaborativa descentralizada basada en consenso usando la fórmula:

```
x̂ᶜᵢ(t) = x̂ᵢ(t) + Σⱼ∈Nᵢ Kᵢⱼ(x̂ⱼ(t) − x̂ᵢ(t))
```

Donde:
- `x̂ᶜᵢ(t)` - Estimación de pose colaborativa para el robot i
- `x̂ᵢ(t)` - Estimación de pose AMCL local
- `Kᵢⱼ` - Ganancia de Kalman (calculada dinámicamente basada en covarianza)
- `Nᵢ` - Conjunto de robots vecinos

### Protocolo de Comunicación

- **Arquitectura**: Grafo completamente conectado (todos los robots se comunican entre sí)
- **Protocolo**: Consenso basado en gossip
- **Tópicos**: `/tb3_X/coloc_pose` y `/tb3_X/coloc_belief`
- **Tasa de actualización**: ~10 Hz

## 🐛 Solución de Problemas

### Problema: Los robots no se mueven

**Causa**: Terminales no iniciadas en el orden correcto o AMCL no inicializado.

**Solución**:
1. Espera "Managed nodes are active" en Terminal 2
2. Asegúrate de que Terminal 3 esté ejecutándose antes de Terminal 4
3. Inicia Terminal 5 al final

### Problema: Errores de localización grandes

**Causa**: El filtro de partículas AMCL aún no ha convergido.

**Solución**: Espera 10-15 segundos después de iniciar Terminal 5 para que las partículas converjan.

### Problema: Colisiones de robots con obstáculos

**Causa**: Parámetro de radio del robot demasiado pequeño en el planificador de rutas.

**Solución**: Aumenta `ROBOT_RADIUS` en `pathplan.py` (línea 91):
```python
ROBOT_RADIUS = 0.20  # Aumentar de 0.12 a 0.20
```

### Problema: Errores "File not found"

**Causa**: Paquete no compilado o entorno no cargado.

**Solución**:
```bash
cd ~/ids_roswk
colcon build --packages-select localization_evaluation
source install/setup.bash
```

---

**Última Actualización**: Enero 2026

