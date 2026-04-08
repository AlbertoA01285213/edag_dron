import streamlit as st
import folium
from streamlit_folium import st_folium
import subprocess
import sys
import yaml
import os
import time

# --- CONFIGURACIÓN DE LA PÁGINA ---
st.set_page_config(layout="wide", page_title="Panel de Control - Dron Ensambladora")

st.title("🚁 Panel de Control de Escaneo de Estacionamiento")
st.markdown("""
Esta aplicación genera el plan de misión para el dron en el simulador. 
1. **Define los parámetros** en la barra lateral.
2. **Haz clic en el mapa** para establecer los puntos GPS.
3. **Genera la configuración** para iniciar ROS 2.
""")

# --- BARRA LATERAL (SIDEBAR) - PARÁMETROS MECATRÓNICOS ---
st.sidebar.header("🔧 Parámetros del Estacionamiento")

# sliders para dimensiones (valores realistas en metros)
cajon_x = st.sidebar.slider("Ancho del cajón (metros)", min_value=2.0, max_value=4.0, value=2.5, step=0.1)
cajon_y = st.sidebar.slider("Largo del cajón (metros)", min_value=4.0, max_value=7.0, value=5.0, step=0.1)
separacion_filas = st.sidebar.slider("Separación entre filas (metros)", min_value=5.0, max_value=15.0, value=8.0, step=0.5)

st.sidebar.markdown("---")
st.sidebar.header("🛸 Parámetros de Vuelo")
altura = st.sidebar.slider("Altura de vuelo (metros)", min_value=3, max_value=20, value=10)
velocidad = st.sidebar.slider("Velocidad de escaneo (m/s)", min_value=0.5, max_value=5.0, value=1.5)
yaw_planeado = st.sidebar.slider("Orientación del dron (Yaw en grados)", min_value=-180, max_value=180, value=0)
duracion_foto = st.sidebar.slider("Tiempo de espera por foto (seg)", min_value=1.0, max_value=10.0, value=3.0)

# Datos de la "matriz" de estacionamiento (asumimos un rectángulo por ahora)
st.sidebar.markdown("---")
cajones_por_fila = st.sidebar.number_input("Cajones por fila", min_value=1, value=10)
num_filas = st.sidebar.number_input("Número de filas", min_value=1, value=2)

# --- MAPA INTERACTIVO (CUERPO PRINCIPAL) ---
st.header("🗺️ Definición de Puntos de Misión")

# Coordenadas iniciales por defecto (Ensambladora Kia Pesquería, NL, México)
# ¡Cámbialas por las coordenadas de tu patio de pruebas si quieres!
START_LAT = 25.779
START_LON = -100.063

# Usamos st.session_state para recordar los puntos clicked entre recargas de página
if 'takeoff_coord' not in st.session_state:
    st.session_state['takeoff_coord'] = None
if 'scan_origin_coord' not in st.session_state:
    st.session_state['scan_origin_coord'] = None

# Crear el mapa base de Folium (usamos satélite de Google para realismo)
m = folium.Map(
    location=[START_LAT, START_LON], 
    zoom_start=17, 
    tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', 
    attr='Google'
)

# Añadir marcadores si ya fueron seleccionados
if st.session_state['takeoff_coord']:
    folium.Marker(
        st.session_state['takeoff_coord'], 
        popup="🚀 Punto Despegue", 
        icon=folium.Icon(color='green', icon='home')
    ).add_to(m)

if st.session_state['scan_origin_coord']:
    folium.Marker(
        st.session_state['scan_origin_coord'], 
        popup="📍 Origen Escaneo", 
        icon=folium.Icon(color='red', icon='flag')
    ).add_to(m)

# El componente mágico que captura el clic
# Captura clics y actualiza el mapa
map_data = st_folium(m, width=900, height=500)

col1, col2 = st.columns(2)

with col1:
    if st.button("🔴 Set: Punto de Despegue"):
        if map_data['last_clicked']:
            st.session_state['takeoff_coord'] = [map_data['last_clicked']['lat'], map_data['last_clicked']['lng']]
            st.rerun() # Recargar página para mostrar marcador
        else:
            st.warning("Haz clic primero en el mapa.")

    if st.session_state['takeoff_coord']:
        st.success(f"Punto Despegue: {st.session_state['takeoff_coord']}")

with col2:
    if st.button("🔵 Set: Origen Escaneo"):
        if map_data['last_clicked']:
            st.session_state['scan_origin_coord'] = [map_data['last_clicked']['lat'], map_data['last_clicked']['lng']]
            st.rerun() # Recargar página para mostrar marcador
        else:
            st.warning("Haz clic primero en el mapa.")
            
    if st.session_state['scan_origin_coord']:
        st.success(f"Origen Escaneo: {st.session_state['scan_origin_coord']}")

if st.button("🗑️ Resetear Puntos"):
    st.session_state['takeoff_coord'] = None
    st.session_state['scan_origin_coord'] = None
    st.rerun()

st.markdown("---")

# --- GENERACIÓN DE ARCHIVO YAML ---
st.header("💾 Generar Configuración")

# Botón para Iniciar Misión
if st.button("🚀 Generar YAML e INICIAR MISIÓN"):
    if st.session_state['takeoff_coord'] and st.session_state['scan_origin_coord']:
        
        # Estructura de la misión para el Mission Handler de ROS 2
        mission_data = {
            'mission_parameters': {
                'timestamp': time.time(),
                'status': 'ready',
                'map_reference': 'WGS84'
            },
            'points': {
                'takeoff': {
                    'lat': st.session_state['takeoff_coord'][0],
                    'lon': st.session_state['takeoff_coord'][1],
                    'alt_relative': 0.0
                },
                'scan_origin': {
                    'lat': st.session_state['scan_origin_coord'][0],
                    'lon': st.session_state['scan_origin_coord'][1],
                    'alt_relative': altura
                }
            },
            'scan_configuration': {
                'type': 'rectangle_grid',
                'cajon_ancho_x': cajon_x,
                'cajon_largo_y': cajon_y,
                'separacion_filas': separacion_filas,
                'total_cajones_fila': cajones_por_fila,
                'total_filas': num_filas,
                'vuelo_altura': altura,
                'vuelo_velocidad': velocidad,
                'dron_yaw': yaw_planeado,
                'foto_duracion_espera': duracion_foto
            }
        }

    
        # CORRECCIÓN: Ruta completa para guardar el YAML en tu paquete de ROS 2
        # Asumo que tu paquete de master está en edag_dron/src/master/config/
        yaml_filename = os.path.expanduser('~/Documents/edag_dron/src/master/config/mision_actual.yaml')
        
        try:
            # Crear carpeta config si no existe
            os.makedirs(os.path.dirname(yaml_filename), exist_ok=True)
            
            with open(yaml_filename, 'w') as file:
                yaml.dump(mission_data, file, default_flow_style=False)
            
            st.success(f"✅ ¡Misión Generada! Archivo YAML guardado en: {yaml_filename}")
            st.json(mission_data) # Mostrar datos generados para depurar

            # comando = f"ros2 launch master mission_handler.py"
            # subprocess.run(['bash', f"{'home/alberto/Documents/edag_dron'}"])
            
            # --- AQUÍ IRÍA LA CONEXIÓN MECATRÓNICA ---
            # En la versión final, Streamlit usaría 'subprocess' para lanzar ROS 2
            # comando = f"ros2 launch master mission_handler_launch.py config_file:={yaml_filename}"
            # st.info(f"Comando que se ejecutaría en segundo plano: {comando}")
            
        except Exception as e:
            st.error(f"Error guardando el archivo YAML: {e}")
            
    else:
        st.error("❌ Por favor, define el Punto de Despegue y el Origen de Escaneo en el mapa primero.")