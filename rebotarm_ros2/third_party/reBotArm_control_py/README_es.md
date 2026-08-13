# Guía de inicio de Pinocchio y MeshCat para el reBot Arm B601-DM

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="Licencia: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Versión de Python">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Plataforma">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>Brazo robótico de 6 DoF · Compatibilidad multimotor · Solucionador cinemático · Planificación de trayectorias · Totalmente de código abierto</strong>
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./README_JP.md">日本語</a>&nbsp;|&nbsp;
    <a href="./README_Fr.md">français</a>&nbsp;|&nbsp;
    <a href="./README_es.md">Español</a>
  </strong>
</p>

---

## 📖 Introducción

**reBotArm Control** es una biblioteca de control en Python para el brazo robótico reBot Arm B601, que proporciona una solución completa desde el control de motores de bajo nivel hasta el cálculo cinemático de alto nivel.

### ✨ Características principales

- 🦾 **Compatibilidad con dos modelos** — B601-DM (motores Damiao) y B601-RS (motores RobStride)
- 🧮 **Solucionador cinemático** — Cinemática directa e inversa basada en Pinocchio
- 🛤️ **Planificación de trayectorias** — Trayectoria geodésica en SE(3) + seguimiento CLIK
- 🔧 **Configuración flexible** — Fichero de configuración YAML para adaptar el hardware rápidamente

---

## ⚙️ Inicio rápido

### Requisitos

| Elemento | Requisito |
|----------|-----------|
| **Python** | 3.10+ |
| **Sistema operativo** | Ubuntu 22.04+ |
| **Interfaz de comunicación** | Puente serie USB2CAN o interfaz CAN |

### Pasos de instalación

#### Paso 1. Instalar uv (si no está instalado)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### Paso 2. Sincronizar el entorno (instalar todas las dependencias)

```bash
git clone https://github.com/vectorBH6/reBotArm_control_py.git
cd reBotArm_control_py
uv sync
```

:::tip
`uv sync` creará automáticamente un entorno virtual (si no existe) e instalará todas las dependencias según `pyproject.toml` y `uv.lock`.
:::

---

## 🔌 Configuración del hardware

### Predeterminado: puente serie USB2CAN de Damiao

El reBot Arm B601-DM utiliza por defecto el módulo de puente serie USB2CAN de Damiao.

**Conexión del hardware**:
1. Conecta el módulo USB2CAN a tu ordenador mediante un cable USB
2. El sistema lo reconocerá automáticamente como el dispositivo `/dev/ttyACM0`

**Verificación de la configuración**:
```bash
# Comprobar el dispositivo
ls /dev/ttyACM0

# Escanear los motores
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### Opcional: interfaz CAN estándar

Si usas otros adaptadores USB-CAN (CANable, PCAN, etc.):

```bash
# Activar la interfaz CAN
sudo ip link set can0 up type can bitrate 500000

# Verificar la interfaz
ip -details link show can0
```

### Configuración según la marca del motor

| Marca del motor | Transmisión | Configuración | Velocidad en baudios |
|-----------------|-------------|---------------|----------------------|
| **Damiao** | Puente serie | `dm-serial` | 921600 |
| **Damiao** | Interfaz CAN | `socketcan` | 500000 |
| **RobStride** | Interfaz CAN | `socketcan` | 500000 |

:::tip
- Con motores Damiao mediante puente serie es obligatorio establecer `--transport dm-serial`
- Regla del ID de realimentación (feedback): `feedback_id = motor_id + 0x10`
:::

---

## 📁 Estructura del proyecto

```
reBotArm_control_py/
├── config/                     # Ficheros de configuración
│   └── robot.yaml              # Configuración de parámetros de las articulaciones
├── example/                    # Programas de ejemplo
│   ├── Debug Tools/            # Herramientas de depuración
│   │   ├── 1_damiao_text.py        # Consola de motor único
│   │   └── 2_zero_and_read.py      # Calibración del cero
│   ├── Kinematics Tests/       # Pruebas de cinemática
│   │   ├── 5_fk_test.py            # Cinemática directa
│   │   └── 6_ik_test.py            # Cinemática inversa
│   ├── Real Machine Control/   # Control del robot real
│   │   ├── 7_arm_ik_control.py     # Control IK en tiempo real
│   │   ├── 8_arm_traj_control.py   # Planificación de trayectorias
│   │   └── 9_gravity_compensation.py  # Compensación de gravedad
│   └── sim/                    # Herramientas de simulación
├── reBotArm_control_py/        # Biblioteca principal
│   ├── actuator/               # Módulo de actuadores
│   ├── kinematics/             # Módulo de cinemática
│   ├── controllers/            # Módulo de controladores
│   └── trajectory/             # Módulo de planificación de trayectorias
├── urdf/                       # Modelo URDF
└── README.md
```

---

## 🎮 Programas de ejemplo

### Herramientas de depuración

#### 1️⃣ Consola de motor único (`1_damiao_text.py`)

Prueba directa de un solo motor con el SDK motorbridge, con tres modos de control.

**Uso**:
```bash
uv run python example/1_damiao_text.py
```

**Comandos interactivos**:
| Comando | Descripción |
|---------|-------------|
| `mit <pos_deg> [vel kp kd tau]` | Modo MIT |
| `posvel <pos_deg> [vlim]` | Modo POS_VEL |
| `vel <vel_rad_s>` | Modo de velocidad |
| `enable` / `disable` | Habilitar/deshabilitar |
| `set_zero` | Establecer la posición cero |
| `state` | Ver el estado |

---

#### 2️⃣ Calibración del cero y monitorización de ángulos (`2_zero_and_read.py`)

Establece automáticamente el cero de todas las articulaciones y muestra los ángulos articulares en tiempo real.

**Uso**:
```bash
uv run python example/2_zero_and_read.py
```

---

### Pruebas de cinemática

#### 5️⃣ Prueba de cinemática directa (`5_fk_test.py`)

Calcula la pose del efector final a partir de los ángulos de las articulaciones.

**Entrada**: 6 ángulos articulares (grados)

**Salida**:
- Posición del efector final (X, Y, Z) — Unidad: metros
- Matriz de rotación (3×3)
- Ángulos de Euler (Roll/Pitch/Yaw) — Unidad: grados

**Ejemplo**:
```bash
uv run python example/5_fk_test.py
> 0 0 0 0 0 0
> 45 -30 15 -60 90 180
```

---

#### 6️⃣ Prueba de cinemática inversa (`6_ik_test.py`)

Resuelve los ángulos de las articulaciones a partir de la pose deseada del efector final.

**Formato de entrada**:
- Solo posición: `<x> <y> <z>` (metros)
- Posición + orientación: `<x> <y> <z> <roll> <pitch> <yaw>` (grados)

**Ejemplo**:
```bash
uv run python example/6_ik_test.py
> 0.25 0.0 0.15              # Solo posición
> 0.25 0.0 0.15 0 0 0        # Posición + orientación
```

---

### Control del robot real

:::tip Configuración de permisos
Antes de ejecutar los ejemplos de control del robot real, necesitas configurar los permisos de los dispositivos:

```bash
# Dar permisos al dispositivo serie (Damiao USB2CAN)
sudo chmod 666 /dev/ttyACM0

# O para la interfaz CAN (p. ej., can0)
sudo chmod 666 /dev/can0
```
:::

#### 7️⃣ Control IK en tiempo real (`7_arm_ik_control.py`)

Control del efector final en tiempo real basado en el solucionador IK.

**Comandos interactivos**:
| Comando | Descripción |
|---------|-------------|
| `x y z [roll pitch yaw]` | Pose objetivo del efector final |
| `state` | Ver el estado actual/objetivo |
| `pos` | Posición actual del efector final |
| `q/quit/exit` | Salir |

**Uso**:
```bash
uv run python example/7_arm_ik_control.py
> 0.3 0.0 0.2
> 0.3 0.1 0.25 0 0.5 0
```

---

#### 8️⃣ Control con planificación de trayectorias (`8_arm_traj_control.py`)

Planificación de trayectoria geodésica en SE(3) + seguimiento CLIK.

**Formato de entrada**:
```
x y z [roll pitch yaw] [duration]
```

**Parámetros**:
- `x, y, z`: posición objetivo (metros)
- `roll, pitch, yaw`: orientación objetivo (radianes)
- `duration`: duración del movimiento (segundos); por defecto, `2.0`

**Uso**:
```bash
uv run python example/8_arm_traj_control.py
> 0.3 0.0 0.3 0 0.4 0 2.0
```

---

#### 9️⃣ Control con compensación de gravedad (`9_gravity_compensation.py`)

Compensa la gravedad en las articulaciones usando el modelo dinámico de Pinocchio.

**Ley de control**:
```
tau = g(q)          — Prealimentación de gravedad
pos = current motor position  — La posición articular sigue la posición actual del motor
kp = 2,  kd = 1     — Rigidez/amortiguación unificadas para todos los motores
```

**Comportamiento esperado**:
- El brazo robótico puede «flotar» en cualquier postura
- No cae por su propio peso al soltarlo
- Se puede mover manualmente a cualquier posición

**Uso**:
```bash
uv run python example/9_gravity_compensation.py
```

**Salida**:
- Muestra en tiempo real el par esperado de cada articulación (N·m)
- Pulsa `Ctrl+C` para detener y desconectar

---

## 📄 Licencia

Este proyecto es de código abierto bajo la **licencia MIT**.

---

## ☎ Contacto

- **Soporte técnico**: [Abrir una incidencia (issue)](https://github.com/vectorBH6/reBotArm_control_py/issues)
- **Repositorio**: [GitHub](https://github.com/vectorBH6/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 ¡Si este proyecto te resulta útil, danos una estrella (Star)!</strong>
</p>
