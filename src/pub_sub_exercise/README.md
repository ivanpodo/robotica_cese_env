# Publisher and Subscriber

## Pre-requisitos

- Ubuntu 24
- ROS2 Jazzy
  - rclcpp
  - rclpy
  - std_msgs
  - std_srvs

## Descripción

Este paquete contiene dos nodos ROS 2:

- **Nodo Publisher**: Publica un contador en el tópico `/publisher_topic` a una frecuencia configurable (por defecto, 5 Hz). El nodo cuenta con un servicio que permite resetear el contador. Además, la cantidad máxima que publica es configurable por parámetro.
- **Nodo Subscriber**: Se suscribe al tópico `/publisher_topic`. Cuando el contador alcanza un valor configurable (por defecto, 50), llama al servicio del nodo publisher para resetear el contador. El valor en el que se reinicia el contador es configurable por parámetro.

## Parámetros configurables

- **Nodo Publisher**:
  - `frequency` (int32_t, default: 5): Frecuencia de publicación en Hz.
  - `max_count` (int32_t, default: 50): Valor máximo del contador antes de reiniciar (si aplica).

- **Nodo Subscriber**:
  - `reset_at` (int32_t, default: 50): Valor del contador en el que se solicita el reseteo.

## Uso

El paquete incluye un launch file que lanza ambos nodos y permite configurar los parámetros desde la línea de comandos.

### Ejemplo de uso

```bash
ros2 launch pub_sub_exercise pub_sub.launch.py frequency:=5.0 reset_at:=50
```

- `frequency`: Frecuencia de publicación del nodo publisher.
- `reset_at`: Valor del contador en el que el subscriber solicita el reseteo.

## Estructura

- `publisher_node.py`: Nodo que publica el contador y expone el servicio de reseteo.
- `subscriber_node.py`: Nodo que se suscribe y solicita el reseteo.
- `launch/pub_sub.launch.py`: Launch file para lanzar ambos nodos con parámetros configurables.

## Ejecución

Dentro del contenedor de desarollo, ejecutar:

```bash
ros2 launch pub_sub_exercise pub_sub.launch.py
```

Para poder cambiar los parametros, se puede utilizar:

```bash
ros2 launch pub_sub_exercise pub_sub.launch.py frequency:=10 max_count:=100 reset_at:=30
```
