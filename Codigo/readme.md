# 🏎️ Controlador Reactivo para F1TENTH - Proyecto Final

Este proyecto implementa un **controlador reactivo inteligente** para un vehículo autónomo en el simulador **F1TENTH**, utilizando el enfoque **Follow the Gap** mejorado, con capacidades de detección de curvas, ajuste dinámico de velocidad, conteo de vueltas y cronómetro por vuelta.

---

## 📌 Descripción del enfoque

El controlador se basa en el algoritmo **Follow the Gap**, que permite detectar la región más libre del entorno usando datos de LiDAR, y dirigir el vehículo hacia ese punto para evitar obstáculos.

Se ha mejorado con las siguientes funcionalidades adicionales:

- ✅ **Ajuste dinámico de velocidad:**\
  Aumenta la velocidad en rectas y la reduce en curvas pronunciadas.
- ✅ **Detección de curvas:**\
  Se analiza la desviación del mejor punto respecto al centro del escaneo para determinar si es curva o recta.
- ✅ **Conteo automático de vueltas:**\
  Se detecta el paso por un punto de referencia en el mapa para registrar una vuelta completa.
- ✅ **Cronómetro por vuelta:**\
  Se registra el tiempo de cada vuelta y se imprime en consola.

---

## 📂 Estructura del código

```
.
├── control_proyecto.py        # Nodo principal del controlador reactivo
├── launch/
│   └── gym_bridge_launch.py   # Archivo de lanzamiento del entorno simulado
├── config/
│   └── sim.yaml               # Parámetros del simulador y mapa
└── maps/
    ├── <nombre_mapa>.yaml     # Archivos de mapa
    └── <nombre_mapa>.png
```

---

## 🚀 Instrucciones de ejecución

### 1. 🔧 Requisitos previos

- Tener ROS 2 Humble instalado
- Tener instalado el simulador `f1tenth_gym_ros`
- Estar dentro del workspace adecuado (por ejemplo `F1Tenth-Repository`)

### 2. 📦 Compilar el proyecto

```bash
colcon build
source install/setup.bash
```

### 3. 🗀 Verificar mapa

Asegúrate de que el archivo `.yaml` y `.png` del mapa que usarás existen en la carpeta `maps/`. Por ejemplo: `Oschersleben_map.yaml` y `Oschersleben_map.png`.

### 4. 🧠 Ejecutar el simulador

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 5. 🥪 Ejecutar el controlador reactivo

En otra terminal:

```bash
ros2 run controllers control_proyecto
```


---

## 📈 Ejemplo de salida por consola

```
✅ Vuelta #1 completada. Tiempo: 31.45 segundos
✅ Vuelta #2 completada. Tiempo: 29.87 segundos
```

---

## 📬 Contacto

> Autor: Armando Moreira\
> Año: 2025

---

