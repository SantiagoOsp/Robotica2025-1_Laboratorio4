#!/usr/bin/env python3
"""
HMI Manipulador Pincher - Interfaz Tkinter
Interfaz gráfica para control del manipulador con servos Dynamixel AX-12A
"""

import tkinter as tk
from tkinter import ttk, messagebox, Canvas
import threading
import time
import math
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import subprocess
import sys

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36

class PincherHMI:
    def __init__(self, root):
        self.root = root
        self.root.title("HMI Manipulador Pincher - Control Dynamixel AX-12A")
        self.root.geometry("1200x800")
        self.root.configure(bg='#f0f0f0')
        
        # Variables de estado
        self.current_positions = [512, 512, 512, 512, 512]
        self.target_positions = [512, 512, 512, 512, 512]
        self.is_connected = False
        self.is_moving = False
        
        # Configuración del puerto serie
        self.port_name = "/dev/ttyUSB0"
        self.baudrate = 1000000
        self.dxl_ids = [1, 2, 3, 4, 5]
        
        # Poses predefinidas
        self.poses = {
            "HOME": [512, 512, 512, 512, 512],
            "POSE 1": [300, 400, 600, 400, 300],
            "POSE 2": [700, 600, 400, 600, 700],
            "POSE 3": [200, 300, 700, 300, 200],
            "POSE 4": [800, 700, 300, 700, 800]
        }
        
        # Inicializar comunicación con servos
        self.init_dynamixel()
        
        # Crear interfaz
        self.create_widgets()
        
        # Iniciar hilo de actualización
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()
    
    def init_dynamixel(self):
        """Inicializar comunicación con servos Dynamixel"""
        try:
            self.port_handler = PortHandler(self.port_name)
            self.packet_handler = PacketHandler(1.0)
            
            if self.port_handler.openPort():
                if self.port_handler.setBaudRate(self.baudrate):
                    self.is_connected = True
                    print("Puerto serie abierto correctamente")
                else:
                    print("Error al configurar baudrate")
            else:
                print("Error al abrir puerto serie")
        except Exception as e:
            print(f"Error inicializando Dynamixel: {e}")
            self.is_connected = False
    
    def create_widgets(self):
        """Crear todos los widgets de la interfaz"""
        
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 1. Header con información del equipo
        self.create_team_info(main_frame)
        
        # Frame para contenido principal
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True, pady=10)
        
        # Frame izquierdo y derecho
        left_frame = ttk.Frame(content_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # 2. Visualización de posición objetivo
        self.create_target_visualization(left_frame)
        
        # 3. Selección de poses
        self.create_pose_selection(left_frame)
        
        # 4. Valores articulares actuales
        self.create_joint_values(right_frame)
        
        # 5. Visualización de posición actual
        self.create_current_visualization(right_frame)
        
        # 6. Barra de estado
        self.create_status_bar(main_frame)
    
    def create_team_info(self, parent):
        """Crear sección de información del equipo"""
        team_frame = ttk.LabelFrame(parent, text="Información del Equipo", padding=10)
        team_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Crear frame para cada integrante
        members_frame = ttk.Frame(team_frame)
        members_frame.pack(fill=tk.X)
        
        team_members = [
            {"name": "Ana García", "role": "Ingeniera de Control", "id": "12345"},
            {"name": "Carlos López", "role": "Desarrollador ROS", "id": "67890"},
            {"name": "María Rodríguez", "role": "Especialista en Robótica", "id": "11111"}
        ]
        
        for i, member in enumerate(team_members):
            member_frame = ttk.Frame(members_frame)
            member_frame.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10)
            
            # Logo (simulado con canvas)
            logo_canvas = Canvas(member_frame, width=60, height=60, bg='lightblue')
            logo_canvas.pack()
            logo_canvas.create_text(30, 30, text=f"M{i+1}", font=("Arial", 16, "bold"))
            
            # Información
            ttk.Label(member_frame, text=member["name"], font=("Arial", 12, "bold")).pack()
            ttk.Label(member_frame, text=member["role"]).pack()
            ttk.Label(member_frame, text=f"ID: {member['id']}").pack()
    
    def create_target_visualization(self, parent):
        """Crear visualización de posición objetivo"""
        target_frame = ttk.LabelFrame(parent, text="🎯 Posición Objetivo (Última Enviada)", padding=10)
        target_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Canvas para dibujar el robot
        self.target_canvas = Canvas(target_frame, width=400, height=300, bg='white')
        self.target_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Dibujar robot inicial
        self.draw_robot(self.target_canvas, self.target_positions, "blue")
    
    def create_pose_selection(self, parent):
        """Crear panel de selección de poses"""
        pose_frame = ttk.LabelFrame(parent, text="🎮 Seleccionar Pose", padding=10)
        pose_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Frame para botones
        buttons_frame = ttk.Frame(pose_frame)
        buttons_frame.pack(fill=tk.X)
        
        # Crear botones para cada pose
        for i, (pose_name, pose_values) in enumerate(self.poses.items()):
            btn = ttk.Button(
                buttons_frame,
                text=f"{pose_name}\n{pose_values}",
                command=lambda p=pose_values, n=pose_name: self.select_pose(p, n),
                width=20
            )
            btn.pack(side=tk.LEFT, padx=5, pady=5, expand=True, fill=tk.X)
        
        # Botón de parada de emergencia
        emergency_btn = ttk.Button(
            pose_frame,
            text="🛑 PARADA DE EMERGENCIA",
            command=self.emergency_stop,
            style="Emergency.TButton"
        )
        emergency_btn.pack(fill=tk.X, pady=(10, 0))
        
        # Configurar estilo para botón de emergencia
        style = ttk.Style()
        style.configure("Emergency.TButton", background="red", foreground="white")
    
    def create_joint_values(self, parent):
        """Crear panel de valores articulares"""
        joints_frame = ttk.LabelFrame(parent, text="📊 Valores Articulares Actuales", padding=10)
        joints_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Variables para mostrar valores
        self.joint_vars = []
        
        # Crear displays para cada articulación
        for i in range(5):
            joint_frame = ttk.Frame(joints_frame)
            joint_frame.pack(fill=tk.X, pady=2)
            
            ttk.Label(joint_frame, text=f"Motor {i+1}:", width=10).pack(side=tk.LEFT)
            
            var = tk.StringVar(value="512")
            self.joint_vars.append(var)
            
            # Valor actual
            ttk.Label(joint_frame, textvariable=var, font=("Arial", 12, "bold"), 
                     foreground="blue", width=8).pack(side=tk.LEFT, padx=10)
            
            # Barra de progreso visual
            progress = ttk.Progressbar(joint_frame, length=200, mode='determinate')
            progress.pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)
            progress['value'] = 50  # 512 es el centro (50%)
            
            # Guardar referencia para actualizar
            setattr(self, f'progress_{i}', progress)
    
    def create_current_visualization(self, parent):
        """Crear visualización de posición actual"""
        current_frame = ttk.LabelFrame(parent, text="🤖 Posición Actual del Manipulador", padding=10)
        current_frame.pack(fill=tk.BOTH, expand=True)
        
        # Canvas para dibujar el robot
        self.current_canvas = Canvas(current_frame, width=400, height=300, bg='white')
        self.current_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Dibujar robot inicial
        self.draw_robot(self.current_canvas, self.current_positions, "green")
    
    def create_status_bar(self, parent):
        """Crear barra de estado"""
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Indicador de conexión
        self.connection_indicator = Canvas(status_frame, width=20, height=20)
        self.connection_indicator.pack(side=tk.LEFT, padx=5)
        
        # Texto de estado
        self.status_var = tk.StringVar(value="Iniciando sistema...")
        status_label = ttk.Label(status_frame, textvariable=self.status_var)
        status_label.pack(side=tk.LEFT, padx=10)
        
        # Tiempo de última actualización
        self.time_var = tk.StringVar(value="--:--:--")
        time_label = ttk.Label(status_frame, textvariable=self.time_var)
        time_label.pack(side=tk.RIGHT, padx=10)
        
        ttk.Label(status_frame, text="Última actualización:").pack(side=tk.RIGHT)
        
        # Actualizar indicador inicial
        self.update_connection_indicator()
    
    def draw_robot(self, canvas, joint_positions, color):
        """Dibujar representación cinemática del robot"""
        canvas.delete("all")
        
        # Parámetros del robot
        center_x, center_y = 200, 250
        link_lengths = [50, 45, 40, 35, 30]
        
        # Convertir posiciones de servo (0-1023) a ángulos (-150° a +150°)
        angles = [(pos - 512) * 300 / 1023 for pos in joint_positions]
        
        # Dibujar base
        canvas.create_oval(center_x-15, center_y-15, center_x+15, center_y+15, 
                          fill="gray", outline="black", width=2)
        
        # Dibujar eslabones
        current_x, current_y = center_x, center_y
        cumulative_angle = 0
        
        for i, (angle, length) in enumerate(zip(angles, link_lengths)):
            cumulative_angle += math.radians(angle)
            
            # Calcular nueva posición
            new_x = current_x + length * math.cos(cumulative_angle)
            new_y = current_y - length * math.sin(cumulative_angle)  # Y invertido en tkinter
            
            # Dibujar eslabón
            canvas.create_line(current_x, current_y, new_x, new_y, 
                             fill=color, width=4)
            
            # Dibujar articulación
            joint_color = "red" if i == len(angles)-1 else color
            canvas.create_oval(new_x-8, new_y-8, new_x+8, new_y+8, 
                             fill=joint_color, outline="black", width=2)
            
            # Etiqueta de articulación
            canvas.create_text(new_x, new_y-20, text=f"J{i+1}", font=("Arial", 8))
            canvas.create_text(new_x, new_y+20, text=f"{joint_positions[i]}", font=("Arial", 8))
            
            current_x, current_y = new_x, new_y
        
        # Dibujar end-effector
        canvas.create_oval(current_x-10, current_y-10, current_x+10, current_y+10, 
                          fill="orange", outline="black", width=2)
        canvas.create_text(current_x, current_y-25, text="EE", font=("Arial", 8, "bold"))
    
    def select_pose(self, pose_values, pose_name):
        """Seleccionar y enviar pose al manipulador"""
        if self.is_moving:
            messagebox.showwarning("Aviso", "El robot está en movimiento. Espere...")
            return
        
        self.target_positions = pose_values.copy()
        self.send_pose_to_robot(pose_values)
        self.draw_robot(self.target_canvas, self.target_positions, "blue")
        self.update_status(f"Enviando {pose_name}: {pose_values}")
    
    def send_pose_to_robot(self, pose_values):
        """Enviar pose al robot usando tu código existente"""
        if not self.is_connected:
            messagebox.showerror("Error", "No hay conexión con el manipulador")
            return
        
        def send_command():
            try:
                self.is_moving = True
                
                # Usar tu código de control_servo.py
                for dxl_id, goal in zip(self.dxl_ids, pose_values):
                    # Configurar parámetros
                    self.packet_handler.write2ByteTxRx(
                        self.port_handler, dxl_id, ADDR_TORQUE_LIMIT, 1000)
                    self.packet_handler.write2ByteTxRx(
                        self.port_handler, dxl_id, ADDR_MOVING_SPEED, 100)
                    
                    # Habilitar torque
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 1)
                    
                    # Enviar posición objetivo
                    self.packet_handler.write2ByteTxRx(
                        self.port_handler, dxl_id, ADDR_GOAL_POSITION, goal)
                
                # Esperar movimiento
                time.sleep(2.0)
                
                # Opcional: Deshabilitar torque
                # for dxl_id in self.dxl_ids:
                #     self.packet_handler.write1ByteTxRx(
                #         self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                
                self.is_moving = False
                self.update_status("Movimiento completado")
                
            except Exception as e:
                self.is_moving = False
                self.update_status(f"Error: {str(e)}")
                messagebox.showerror("Error", f"Error enviando comando: {str(e)}")
        
        # Ejecutar en hilo separado
        threading.Thread(target=send_command, daemon=True).start()
    
    def read_current_positions(self):
        """Leer posiciones actuales de los servos"""
        if not self.is_connected:
            return
        
        try:
            for i, dxl_id in enumerate(self.dxl_ids):
                pos, _, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, dxl_id, ADDR_PRESENT_POSITION)
                if pos is not None:
                    self.current_positions[i] = pos
        except Exception as e:
            print(f"Error leyendo posiciones: {e}")
    
    def emergency_stop(self):
        """Parada de emergencia"""
        if messagebox.askyesno("Confirmación", "¿Ejecutar parada de emergencia?"):
            try:
                for dxl_id in self.dxl_ids:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.is_moving = False
                self.update_status("🛑 PARADA DE EMERGENCIA EJECUTADA")
            except Exception as e:
                messagebox.showerror("Error", f"Error en parada de emergencia: {str(e)}")
    
    def update_status(self, message):
        """Actualizar mensaje de estado"""
        self.status_var.set(message)
        self.time_var.set(time.strftime("%H:%M:%S"))
    
    def update_connection_indicator(self):
        """Actualizar indicador de conexión"""
        self.connection_indicator.delete("all")
        color = "green" if self.is_connected else "red"
        self.connection_indicator.create_oval(2, 2, 18, 18, fill=color, outline="black")
    
    def update_joint_displays(self):
        """Actualizar displays de valores articulares"""
        for i, pos in enumerate(self.current_positions):
            self.joint_vars[i].set(str(int(pos)))
            # Actualizar barra de progreso (0-1023 -> 0-100%)
            progress_value = (pos / 1023) * 100
            getattr(self, f'progress_{i}')['value'] = progress_value
    
    def update_loop(self):
        """Bucle principal de actualización"""
        while True:
            try:
                # Leer posiciones actuales
                self.read_current_positions()
                
                # Actualizar interfaz en el hilo principal
                self.root.after(0, self.update_displays)
                
                time.sleep(0.1)  # Actualizar cada 100ms
                
            except Exception as e:
                print(f"Error en update_loop: {e}")
                time.sleep(1)
    
    def update_displays(self):
        """Actualizar displays en el hilo principal"""
        self.update_joint_displays()
        self.draw_robot(self.current_canvas, self.current_positions, "green")
        self.update_connection_indicator()
        
        if not self.is_moving:
            self.update_status("Sistema listo")
    
    def on_closing(self):
        """Manejar cierre de la aplicación"""
        if messagebox.askokcancel("Salir", "¿Desea salir de la aplicación?"):
            try:
                # Deshabilitar todos los servos
                for dxl_id in self.dxl_ids:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, dxl_id, ADDR_TORQUE_ENABLE, 0)
                
                # Cerrar puerto
                if hasattr(self, 'port_handler'):
                    self.port_handler.closePort()
                    
            except Exception as e:
                print(f"Error al cerrar: {e}")
            finally:
                self.root.destroy()

def main():
    """Función principal"""
    # Verificar dependencias
    try:
        import rclpy
        from dynamixel_sdk import PortHandler, PacketHandler
    except ImportError as e:
        print(f"Error: Dependencia faltante - {e}")
        print("Instale las dependencias necesarias:")
        print("pip install dynamixel-sdk")
        print("sudo apt install ros-humble-desktop")
        return
    
    # Crear aplicación
    root = tk.Tk()
    app = PincherHMI(root)
    
    # Configurar cierre de ventana
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # Ejecutar aplicación
    root.mainloop()

if __name__ == "__main__":
    main()
