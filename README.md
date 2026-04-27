# edag_dron
## Description
El objetivo de este proyecto es desarrollar un dron que sea capaz de escanear los carros en los estacionamientos. El dron vera la posicion del carro y la informacion en el parabrisas. Esos datos los desplegara en una base de datos. 
Esto con el fin de automatizar la tarea que haria un grupo de personas a mano. Reduciendo fatiga, desperdicio de tiempo y ahorrando dinero.

## Funcionamiento
Se tienen 3 partes para el funcionamiento: La aplicacion, ROS2 y PX4. Cada una tiene sus funciones. Empecemos con la PX4

### PX4
La PX4 o PixHawk 4 es la computadora de vuelo para drones mas famosa. Esta se encarga de controlar el dron. Recibe los comandos de movimiento y la PX4 los traduce a la potencia de los motores para que el dron pueda moverse y mantenerse en el aire. En este caso usamos PX4_SITL (Software In The Loop), la variante para simulaciones donde la misma computadora simula la PixHawk con sus entradas y salidas. Adicionalmente PX4_SITL incluye la visualizacion de la simulacion de gazebo.

### ROS2
ROS2 es el software intermediario que dicta las ordenes para el dron. Este funciona con nodos que son los codigos y topicos que son los canales de comunicacion entre codigos. El ROS2 tiene un nodo **administrador de misiones** el cual es el centro de la red de ROS2. Este determina las funciones principales como despegar, aterrizar o hacer ciertas cosas. Este recibe informacion de otros nodos y topicos de gazebo para poder tomar las decisiones en base a una mision predefinida.

### Aplicacion
La aplicacion es la interfaz que el cliente vera. Disenada con StreamLit utiliza LeafLet para que en un mapa puedas poner las coordenadas exactas de despegue. Adicionalmente insertas variables como dimensiones de cajones y estacionamiento. La aplicacion creara un archivo yaml que el ROS2 pueda leer y ajustarse para que el dron pueda hacer su rutina.  

Adicionalmente se vera si con otra aplicacion se pueda visualizar la base de datos que el dron genere.

## Requisitos
La computadora en la que corra el programa debera de tener:
- Ubuntu 22.04
- ROS2 Humble
- Git

## Como se ejecuta
Para ejecutar la aplicacion se deben de seguir los siguientes pasos:  
(Nota: De preferencia hacer el clone dentro de la carpeta *Documents*)
1. Clonar el repositorio
```
cd Documents
git clone https://github.com/AlbertoA01285213/edag_dron.git
```
2. Clonar PX4 (No se si al clonar el repo se agreguen estos, si no hay que clonarlo)
```
cd src
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
https://github.com/PX4/px4_msgs.git
```
3. Preparar el ROS2
```
cd ..
colcon build
source install/setup.bash
```
4. Correr todo
```
ros2 launch master gazebo_launch.py
```
En otra terminal
```
streamlit run app_dron.py
```
Listo ya tendras todo funcionando.

## Tareas
- **Mejorar la aplicacion** Desarrollarla para que sea mas intuitiva de usar, puedas guardar posiciones predefinidas. Que el mapa no se este reiniciando cada vez que se pone un punto.
- **Incluir repos al repo** Hay que incluir los repositorios *PX4-Autopilot*, *Micro-XRCE-DDS-Agent* y *px4_msgs* en el repo. Creo que esto se hace por medio de forks ya que hay unas modificaciones, sobretodo en el autopilot.
- **Incluir limitante de velocidad** En el launchfile incluir *param set MPC_XY_VEL_MAX 2.0* cuando se abra le px4_sitl. Este comando limita la velocidad del dron. Podria hacerse tambien que en la aplicacion puedas definir ese parametro.
- **Hardcodes** Por ahora solo hay una cosa hardcodeada a la computadora de Alberto. En el nodo de *foto.py* en vision hay una funcion que guarda las fotos en */Desktop/Fotos_dron*. Pero tiene el enlace root que hace referencia a mi computadora. Si quieren que funcione para la suya hacer el cambio de root. Mas adelante eliminar esa seccion.
- **Parametro para launchfile** Agregar un parametro al launchfile para activar o desactivar la interaccion con la aplicacion. Si *True* se usan los parametros del yaml que hace la aplicacion. Si *False* se unan los parametros predefinidos para debugging.
- **Implementacion de DJI** Ver como se puede controlar un DJI con ROS2 y agregar esas modificaciones al programa. Esto con el fin de utilizar la px4 como gemelo digital y el control de DJI como ejecutor en vida real.
- **Mejorar comando de PX4 del launchfile** El comando que realiza el make de la px4 esta basico. Se pueden insertar las coordenadas donde iniciara el dron y varias cosas mas que se ocuparan para implementar la aplicacion.
- **Adaptar ROS2 a la aplicacion** Se ocupan modificar los nodos para que lean el yaml que tiene los parametros para arrancar el dron. Revisar *mission_handler.py* y no se que nodos mas.
- **Dockerfile** Al final realizar y si se puede realizar un dockerfile para empaquetar todo para que pueda correr en cualquier computadora.
- **Cambiar StreamLit a React** Al final y si se puede migrar la aplicacion de StreamLit a React. Esto para hacerla mas profesional y que sea aplicacion y no websocket.

## Problemas conocidos
- No se pudo obtener el bridge GZ-ROS2 para el topico de la camara. Por lo que se hizo un pequeno script que convierte el feed de la camara de gazebo a transmision UDP. Por lo que hay un nodo de ROS2 que se suscribe a esa transmision y asi se obtiene el feed en ROS2. Por lo que no hay bridge para eso.
- La simulacion de PX4-SITL no calibra bien los sensores. Por lo que constantemente salen mensajes de error. Se pueden ignorar, si estan bien calibrados y funcionan, solo que la PX4 no lo sabe.
- Si usas ubuntu nativamente, el gazebo puede causar problemas. Especificamente se pondra la GUI en negro y te pedira forzar el cierre. Es un problema de multicast. [Revisar este link](https://docs.ros.org/en/rolling/How-To-Guides/Installation-Troubleshooting.html#enable-multicast)


## Debugging
### Si quieres correr todo manualmente
(Teniendo en cuenta que ya estas dentro del paquete clonado)  

Terminal 1:
```
streamlit run app_dron.py
```

Terminal 2:
```
colcon build
source install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3:
```
cd src
cd PX4-Autopilot
make px4_sitl gz_x500
param set MPC_XY_VEL_MAX 2.0
```

Terminal 4:
```
cd src
cd vision
cd scripts
python3 camera.py
```

Terminal 5:
```
source install/setup.bash
ros2 run vision receptor_udp
```

Terminal 6:
```
source install/setup.bash
ros2 run master mission_handler
```

### Si quieres ver topicos
Topicos del gazebo
```
gz topic -l           # Ver la lista de topicos
gz topic -e -t /      # Ver la informacion de cierto topico
```

Topicos de ROS2
```
ros2 topic list                 # Ver la lista de topicos
ros2 topic echo /               # Ver la informacion de cierto topico
rqt_graph                       # Ver las relaciones entre nodos
ros2 run rqt_image_view rqt_image_view   # Ver camaras
```

Mover el dron en PX4_SITL
```
commander arm -f           # Armar el dron (Siempre inicia armado)
commander takeoff          # Despegar el dron
commander land             # Aterrizar el dron
```

### Modificar los modelos de la simulacion
Para poner nuestro dron y mundo en la simulacion de PX4_SITL se hizo una mexicanada para no batallar. Se modificaron los archivos basicos del dron y mundo que vienen por defecto.  

Si se desea modificar los modelos se puede hacer en los siguientes directorios:  
Para el dron:
```
# Agregar algun modelo
/src/PX4-Autopilot/Tools/simulation/gz/models/dron/
# Modificar alguna dimension o sensor
/src/PX4-Autopilot/Tools/simulation/gz/models/x_500_base/model.sdf
```
Para el mundo:
```
# Agregar algun modelo
/src/PX4-Autopilot/Tools/simulation/gz/models/estacionamiento/
# Modificar alguna dimension o sensor
/src/PX4-Autopilot/Tools/simulation/gz/worlsds/default.sdf
```

## Comandos basicos de ubuntu
Antes que nada instalen terminator, les ayudara a tener las terminales mas ordenadas y poder abrir mas de manera mas facil. [Instalar aqui](https://innovativeinnovation.github.io/ubuntu-setup/terminals/terminator.html).

### Comandos en terminal
```
# Comandos para terminales
ctrl + alt + t                  # Abrir terminal
ctrl + shift + e                # Partir terminal verticalmente
ctrl + shift + o                # Partir terminal horizontalmente
exit                            # Cerrar terminal

# Comandos basicos de la terminal
cd Desktop                      # Ir a carpeta
cd ..                           # Regresarse a la carpeta anterior
cd                              # Regresar a carpeta principal
rm -rf Documents                # Eliminar cierta carpeta o archivo
tab + tab                       # Autocompletar (no siempre se puede)

# Comandos de ROS2
colcon build                    # Compilar todos los paquetes
colcon build --packages-select  # Compilar cierto paquete
source install/setup.bash       # Activar ROS2 en la terminal

# Comandos de PX4
commander takeoff               # Despegar dron
commander land                  # Aterrizar dron
commander arm -f                # Armar dron
```

## Notas
Aqui pueden poner notas y observaciones para que todos estemos al tanto de lo que esta pasando, detalles, modificaciones y cualquier cosa que se ocupe. Obvio avisar en whatsapp tambien para darnos cuenta.

COM_DISARM_PRFLT
param set MPC_XY_VEL_MAX 1.5
param set MC_YAWRATE_MAX 70