# pincher_control/gui_control.py
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import os
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import threading
import time

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36

class DynamixelGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Control de Servomotores Dynamixel")
        self.root.geometry("1200x400")  # Aumenté la altura de la ventana
        self.root.configure(bg='#f0f0f0')
       
        # Configuraciones en grados
        self.configurations = {
            "Home": (0, 0, 0, 0, 0),
            "Posicion 1": (25, 25, 20, -20, 0),
            "Posicion 2": (-35, 35, -30, 30, 0),
            "Posicion 3": (85, -20, 55, 25, 0),
            "Posicion 4": (80, -35, 55, -45, 0)
        }
       
        # Parámetros de conexión
        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 57600
        self.dxl_ids = [1, 2, 3, 4, 5]
        self.moving_speed = 100
        self.torque_limit = 1000
       
        # Variables de estado
        self.connected = False
        self.port = None
        self.packet = None
        self.position_update_active = False
       
        # Lista para mantener referencias a las imágenes
        self.image_references = []
       
        self.setup_ui()
   
    def setup_ui(self):
        # Título principal
        title_label = tk.Label(
            self.root,
            text="Integrantes Grupo",
            font=("Arial", 16, "bold"),
            bg='#f0f0f0',
            fg='#333333'
        )
        title_label.pack(pady=10)
       
        # Frame para parámetros de conexión
        connection_frame = tk.Frame(self.root, bg='#f0f0f0')
        connection_frame.pack(pady=10)
       
        tk.Label(connection_frame, text="Santiago Ospina", bg='#f0f0f0').grid(row=0, column=0, padx=5)
        
        tk.Label(connection_frame, text="Juan Diego Tovar", bg='#f0f0f0').grid(row=0, column=2, padx=5)
      
        # Botón de conexión
        self.connect_button = tk.Button(
            connection_frame,
            text="Conexion",
            command=self.toggle_connection,
            bg='#4CAF50',
            fg='white',
            font=("Arial", 10, "bold")
        )
        self.connect_button.grid(row=0, column=4, padx=10)
       
        # Frame principal para las configuraciones
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(pady=20, padx=20, fill='both', expand=True)
       
        # Crear botones para cada configuración
        self.create_configuration_buttons(main_frame)
       
        # Frame para información de estado
        status_frame = tk.Frame(self.root, bg='#f0f0f0')
        status_frame.pack(pady=10, fill='x')
       
        self.status_label = tk.Label(
            status_frame,
            text="Estado: Desconectado",
            bg='#f0f0f0',
            fg='#f0f0f0',
            font=("Arial", 10)
        )
        self.status_label.pack()
       
    def get_script_directory(self):
        """Obtener el directorio donde se encuentra el script actual"""
        try:
            # Si se ejecuta como script
            return os.path.dirname(os.path.abspath(__file__))
        except:
            # Si se ejecuta de otra manera
            return os.getcwd()
   
    def find_ros2_workspace(self):
        """Encontrar el workspace de ROS2 automáticamente"""
        current_dir = os.getcwd()
       
        # Buscar hacia arriba hasta encontrar un directorio que contenga 'src'
        search_dir = current_dir
        while search_dir != '/':
            if os.path.exists(os.path.join(search_dir, 'src')):
                return search_dir
            search_dir = os.path.dirname(search_dir)
       
        # Si no encontramos workspace, intentar rutas comunes
        common_workspaces = [
            os.path.expanduser("~/ros2_ws"),
            os.path.expanduser("~/ros2_ws/phantom_ws"),
            os.path.expanduser("~/phantom_ws"),
            "/home/miguel/ros2_ws/phantom_ws"
        ]
       
        for ws in common_workspaces:
            if os.path.exists(ws) and os.path.exists(os.path.join(ws, 'src')):
                return ws
       
        return None
   
    def find_image_file(self, relative_path):
        """Buscar archivo de imagen en múltiples ubicaciones posibles"""
        script_dir = self.get_script_directory()
        ros2_ws = self.find_ros2_workspace()
       
        # Lista de ubicaciones posibles para buscar las imágenes
        possible_paths = [
            # Ruta relativa desde el script actual
            os.path.join(script_dir, relative_path),
            # Ruta relativa desde el directorio de trabajo actual
            os.path.join(os.getcwd(), relative_path),
            # Ruta expandida del usuario
            os.path.expanduser(relative_path),
            # Ruta absoluta si se proporciona
            relative_path if os.path.isabs(relative_path) else None,
        ]
       
        # Agregar rutas específicas del workspace ROS2 si se encuentra
        if ros2_ws:
            possible_paths.extend([
                os.path.join(ros2_ws, "src/pincher_control/pincher_control", relative_path),
                os.path.join(ros2_ws, "src/pincher_control", relative_path),
            ])
       
        # Rutas de fallback específicas para este proyecto
        possible_paths.extend([
            "/home/miguel/ros2_ws/phantom_ws/src/pincher_control/pincher_control/" + relative_path,
            os.path.join(os.path.expanduser("~/ros2_ws/phantom_ws/src/pincher_control/pincher_control"), relative_path),
        ])
       
        # Filtrar None y verificar existencia
        for path in filter(None, possible_paths):
            if os.path.exists(path):
                print(f"Imagen encontrada en: {path}")
                return path
       
        print(f"Rutas buscadas para {relative_path}:")
        for path in filter(None, possible_paths):
            print(f"  - {path} {'(EXISTE)' if os.path.exists(path) else '(NO EXISTE)'}")
       
        return None
   
    def load_image(self, image_path, size=(180, 350), rotate_degrees=90):
        """Cargar y redimensionar imagen con mejor manejo de errores y rotación - ALTURA 3X ANCHO"""
        try:
            # Buscar el archivo en múltiples ubicaciones
            actual_path = self.find_image_file(image_path)
           
            if actual_path is None:
                print(f"Advertencia: No se encontró la imagen en ninguna ubicación para: {image_path}")
                # Intentar buscar en subdirectorio img del script
                script_dir = self.get_script_directory()
                img_dir = os.path.join(script_dir, 'img')
                if os.path.exists(img_dir):
                    print(f"Contenido del directorio img: {os.listdir(img_dir)}")
                return None
           
            print(f"Cargando imagen desde: {actual_path}")
           
            # Cargar imagen
            image = Image.open(actual_path)
           
            # Rotar imagen 90 grados en sentido horario
            if rotate_degrees != 0:
                image = image.rotate(-rotate_degrees, expand=True)  # Negativo para horario
           
            # Redimensionar imagen - Compatible con versiones antiguas y nuevas de Pillow
            try:
                # Intentar con la nueva sintaxis (Pillow >= 10.0.0)
                image = image.resize(size, Image.Resampling.LANCZOS)
            except AttributeError:
                # Fallback para versiones antiguas de Pillow
                try:
                    image = image.resize(size, Image.LANCZOS)
                except AttributeError:
                    # Fallback adicional para versiones muy antiguas
                    image = image.resize(size, Image.ANTIALIAS)
           
            photo_image = ImageTk.PhotoImage(image)
           
            # Guardar referencia para evitar recolección de basura
            self.image_references.append(photo_image)
           
            return photo_image
           
        except Exception as e:
            print(f"Error cargando imagen {image_path}: {e}")
            return None
   
    def create_configuration_buttons(self, parent):
        # Crear grid de 5 columnas para las configuraciones
        configs = list(self.configurations.keys())
       
        # Definir rutas de imágenes para cada configuración
        image_paths = {
            "Home": "img/hoome.jpg",
            "Config 1": "img/Posicion1.jpg",
            "Config 2": "img/Posicion2.jpg",
            "Config 3": "img/Posicion3.jpg",
            "Config 4": "img/posicion4.jpg"
        }
       
        # Mostrar información de debug
        print(f"Directorio del script: {self.get_script_directory()}")
        print(f"Directorio de trabajo: {os.getcwd()}")
        ros2_ws = self.find_ros2_workspace()
        print(f"Workspace ROS2 detectado: {ros2_ws}")
       
        # Verificar si existe el directorio de imágenes
        if ros2_ws:
            img_dir = os.path.join(ros2_ws, "src/pincher_control/pincher_control/img")
            if os.path.exists(img_dir):
                print(f"Contenido del directorio de imágenes: {os.listdir(img_dir)}")
            else:
                print(f"Directorio de imágenes no encontrado en: {img_dir}")
       
        # Verificar ruta absoluta específica
        abs_img_dir = "/home/miguel/ros2_ws/phantom_ws/src/pincher_control/pincher_control/img"
        if os.path.exists(abs_img_dir):
            print(f"Contenido del directorio absoluto: {os.listdir(abs_img_dir)}")
        else:
            print(f"Directorio absoluto no encontrado: {abs_img_dir}")
       
        print("---")
       
        for i, config_name in enumerate(configs):
            # Frame para cada configuración - AUMENTADO EN ALTURA
            config_frame = tk.Frame(parent, bg='#ffffff', relief='raised', bd=2)
            config_frame.grid(row=0, column=i, padx=15, pady=15, sticky='nsew')
           
            # Configurar peso de las columnas
            parent.columnconfigure(i, weight=1)
           
            # Etiqueta con el nombre de la configuración
            name_label = tk.Label(
                config_frame,
                text=config_name,
                bg='#ffffff',
                font=("Arial", 12, "bold")
            )
            name_label.pack(pady=5)
           
            # Mostrar configuración en grados
            degrees_text = str(self.configurations[config_name])
            degrees_label = tk.Label(
                config_frame,
                text=f"Grados: {degrees_text}",
                bg='#ffffff',
                font=("Arial", 9),
                fg='#666666',
                wraplength=160  # Permite que el texto se ajuste
            )
            degrees_label.pack(pady=5)
           
            # Botón para ejecutar configuración
            execute_button = tk.Button(
                config_frame,
                text="Ejecutar",
                command=lambda cfg=config_name: self.execute_configuration(cfg),
                bg="#FF9900",
                fg='white',
                font=("Arial", 12, "bold"),
                width=15,
                height=2
            )
            execute_button.pack(pady=10)
   
    def dynamixel_value_to_degrees(self, dynamixel_value):
        """
        Convierte valor Dynamixel (0-1023) a grados
        La posición home (512) corresponde a 0 grados
        """
        degrees = (dynamixel_value - 512) / 3.41
        return round(degrees, 1)
   
    def degrees_to_dynamixel_value(self, degrees):
        """
        Convierte grados a valor Dynamixel (0-1023)
        La posición home (512) corresponde a 0 grados
        Rango aproximado: -150° a +150° → 0 a 1023
        """
        # Conversión: 1 grado ≈ 3.41 unidades Dynamixel
        # Fórmula: valor = 512 + (grados * 3.41)
        dynamixel_value = int(512 + (degrees * 3.41))
       
        # Limitar valores entre 0 y 1023
        return max(0, min(1023, dynamixel_value))
   
    def toggle_connection(self):
        if not self.connected:
            self.connect_to_dynamixel()
        else:
            self.disconnect_from_dynamixel()
   
    def connect_to_dynamixel(self):
        try:
            self.port_name = '/dev/ttyUSB0'
            self.baudrate = 57600
           
            # Inicializar comunicación
            self.port = PortHandler(self.port_name)
            if not self.port.openPort():
                raise Exception(f"No se pudo abrir el puerto {self.port_name}")
           
            if not self.port.setBaudRate(self.baudrate):
                raise Exception(f"No se pudo establecer el baudrate {self.baudrate}")
           
            self.packet = PacketHandler(1.0)
            self.connected = True
           
            # Actualizar UI
            self.connect_button.config(text="Desconectar", bg='#f0f0f0')
            self.status_label.config(text="Estado: Conectado", fg='#f0f0f0')
           
            messagebox.showinfo("Éxito", "Conectado correctamente a los servomotores")
           
        except Exception as e:
            messagebox.showerror("Error de conexión", f"Error al conectar: {str(e)}")
            self.connected = False
   
    def disconnect_from_dynamixel(self):
        try:
            # Detener actualización de posiciones
            # self.stop_position_updates()
           
            if self.port:
                # Apagar torque en todos los servos antes de desconectar
                for dxl_id in self.dxl_ids:
                    self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
               
                self.port.closePort()
                self.port = None
                self.packet = None
           
            self.connected = False
           
        except Exception as e:
            messagebox.showerror("Error", f"Error al desconectar: {str(e)}")
   
    def execute_configuration(self, config_name):
        if not self.connected:
            messagebox.showwarning("Advertencia", "Primero debe conectarse a los servomotores")
            return
       
        try:
            # Obtener configuración en grados
            degrees_config = self.configurations[config_name]
           
            # Convertir a valores Dynamixel
            dynamixel_values = [self.degrees_to_dynamixel_value(deg) for deg in degrees_config]
           
            # Confirmar ejecución
            degrees_str = ', '.join([f"{deg}°" for deg in degrees_config])
            dynamixel_str = ', '.join([str(val) for val in dynamixel_values])
           
            confirm = messagebox.askyesno(
                "Confirmar movimiento",
                f"Ejecutar configuración: {config_name}\n\n"
                f"Grados: [{degrees_str}]\n"
                f"Valores Dynamixel: [{dynamixel_str}]\n\n"
                f"¿Continuar?"
            )
           
            if not confirm:
                return
           
            # Ejecutar movimiento en un hilo separado
            threading.Thread(
                target=self.move_servos,
                args=(dynamixel_values, config_name),
                daemon=True
            ).start()
           
        except Exception as e:
            messagebox.showerror("Error", f"Error al ejecutar configuración: {str(e)}")
   
    def move_servos(self, goal_positions, config_name):
        try:
            # Actualizar status
            self.root.after(0, lambda: self.status_label.config(
                text=f"Estado: Ejecutando {config_name}...", fg='#f0f0f0'
            ))
           
            # Mover cada servo SECUENCIALMENTE con retraso de 2 segundos
            for i, (dxl_id, goal) in enumerate(zip(self.dxl_ids, goal_positions)):
                # Mostrar qué motor se está moviendo
                self.root.after(0, lambda motor=i+1: self.status_label.config(
                    text=f"Estado: Moviendo Motor {motor} - {config_name}", fg='#f0f0f0'
                ))
               
                # Configurar límites para el servo actual
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_TORQUE_LIMIT, self.torque_limit)
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, self.moving_speed)
               
                # Habilitar torque
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
               
                # Enviar posición objetivo
                self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, goal)
               
                print(f"Motor {dxl_id} movido a posición {goal} ({self.dynamixel_value_to_degrees(goal)}°)")
               
                # Retraso de 2 segundos antes del siguiente motor (excepto el último)
                if i < len(self.dxl_ids) - 1:
                    time.sleep(2.0)
           
            # Esperar un poco más para que se complete el último movimiento
            time.sleep(1.0)
           
            # Actualizar status a conectado
            self.root.after(0, lambda: self.status_label.config(
                text="Estado: Conectado", fg='#f0f0f0'
            ))
           
            # Mostrar mensaje de éxito
            self.root.after(0, lambda: messagebox.showinfo(
                "Éxito",
                f"Configuración '{config_name}' ejecutada correctamente\n"
                f"Movimiento secuencial completado en {len(self.dxl_ids) * 2} segundos"
            ))
           
        except Exception as e:
            self.root.after(0, lambda: self.status_label.config(
                text="Estado: Error", fg='#f0f0f0'
            ))
            self.root.after(0, lambda: messagebox.showerror(
                "Error", f"Error durante el movimiento: {str(e)}"
            ))
   
    def on_closing(self):
        # Detener actualización de posiciones
        # self.stop_position_updates()
       
        if self.connected:
            self.disconnect_from_dynamixel()
        self.root.destroy()

def main():
    # Inicializar ROS2 (opcional, si se necesita)
    # rclpy.init()
   
    root = tk.Tk()
    app = DynamixelGUI(root)
   
    # Configurar cierre de ventana
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
   
    # Ejecutar interfaz
    root.mainloop()
   
    # Finalizar ROS2 (opcional)
    # rclpy.shutdown()

if __name__ == '__main__':
    main()