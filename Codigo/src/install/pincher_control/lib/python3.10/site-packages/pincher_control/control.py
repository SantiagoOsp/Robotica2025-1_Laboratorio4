# pincher_control/control_servo.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

import tkinter as tk, threading

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class Aplicacion:
    """
    Clase que inicaliza y crea la ventana de inicio de sesion.
    """
    def __init__(self, pincher):

        self.pincher = pincher
        # ----------CARACTERISTECAS DE VENTANA-------------------
        self.root = tk.Tk()
        #Obtener medidas de pantalla para centrar ventana.
        self.sw = self.root.winfo_screenwidth()
        self.sh = self.root.winfo_screenheight()
        self.x = self.sw // 3
        self.y = self.sh // 4
        self.root.title("RobotPincher")
        self.root.geometry(f"400x400+{self.x}+{self.y}")
        self.root.resizable(width=False, height=False)
        self.root.config(bg="white")

        #Se crea lienzo.
        self.loginCanva = tk.Canvas(self.root, width=self.sw, height=self.sh, bg='white')
        self.loginCanva.place(x=0, y=0)

        # ------------------BOTÓN POSICION 1--------------------

        self.bPos1 = tk.Button(self.root, text="Posicion 1", font=('Impact', 10), background='#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos1.place(x=80, y=70)

        # ------------------BOTÓN POSICION 2---------------------

        self.bPos2 = tk.Button(self.root, text="Posicion 2",font=('Impact', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos2.place(x=180, y=70)

        # -------------------BOTÓN POSICION 3------------------------
        self.bPos3 = tk.Button(self.root, text="Posicion 3",font=('Impact', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos3.place(x=280,y=70)
        
        # -------------------BOTÓN POSICION 4------------------------
        self.bPos4 = tk.Button(self.root, text="Posicion 4",font=('Impact', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos4.place(x=100, y=130)

        # -------------------BOTÓN POSICION 5------------------------
        self.bPos5 = tk.Button(self.root, text="Posicion 5",font=('Impact', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos5.place(x=200,y=130)

        # --------------MAINLOOP-------------------------
        self.root.mainloop()
    
    # ---------------METODOS ----------------------

    # ------ Posiciones --------------------------
    def Posicion1(self):
        nueva_pos=[512, 512, 512, 512, 512]
        self.pincher.set_goal_positions(nueva_pos)

    def Posicion2(self):
        nuevas_pos = [25, 25, 20, -20, 0]
        self.pincher.set_goal_positions(nuevas_pos)
    
    def Posicion3(self):
        nuevas_pos = [-35, 35, -30, 30, 0]
        self.pincher.set_goal_positions(nuevas_pos)
    
    def Posicion4(self):
        nuevas_pos = [85,-20, 55, 25, 0]
        self.pincher.set_goal_positions(nuevas_pos)
    
    def Posicion5 (self):
        nuevas_pos = [80, -35, 55, -45, 0]
        self.pincher.set_goal_positions(nuevas_pos)


class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [300, 400, 512, 512, 512])
        self.declare_parameter('moving_speed', 100)     # 0–1023 
        self.declare_parameter('torque_limit', 1000)     # 0–1023 
        self.declare_parameter('delay', 2.0)

        port_name      = self.get_parameter('port').value
        baudrate       = self.get_parameter('baudrate').value
        dxl_ids        = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed   = self.get_parameter('moving_speed').value
        torque_limit   = self.get_parameter('torque_limit').value
        delay_seconds  = self.get_parameter('delay').value

        if len(goal_positions) != len(dxl_ids):
            self.get_logger().error(
                f'La lista goal_positions ({len(goal_positions)}) '
                f'debe tener la misma longitud que dxl_ids ({len(dxl_ids)})'
            )
            rclpy.shutdown()
            return

        # Inicializar comunicación
        port   = PortHandler(port_name)
        port.openPort()
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        # 1) Configurar torque_limit, velocidad y enviar posición a cada servo
        for dxl_id, goal in zip(dxl_ids, goal_positions):
            # Limitar torque
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            # Limitar velocidad
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            # Habilitar torque
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            # Enviar posición objetivo
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(f'[ID {dxl_id}] → goal={goal}, speed={moving_speed}, torque_limit={torque_limit}')
            time.sleep(delay_seconds)

        # 2) (Opcional) Leer y mostrar posición actual
        for dxl_id in dxl_ids:
            pos, _, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            self.get_logger().info(f'[ID {dxl_id}] posición actual={pos}')

        # 3) Esperar a que todos los servos alcancen la posición
        self.get_logger().info(f'Esperando {delay_seconds}s para completar movimiento...')
        time.sleep(delay_seconds)

        # 4) Apagar torque en todos los servos
        for dxl_id in dxl_ids:
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        # 5) Cerrar puerto y terminar nodo
        port.closePort()
        rclpy.shutdown()
    
    def set_goal_positions(self, nuevas_posiciones):
            self.set_parameters([rclpy.parameter.Parameter(
                'goal_positions', rclpy.Parameter.Type.INTEGER_ARRAY, nuevas_posiciones)])
            self.get_logger().info(f"Nueva posición objetivo: {nuevas_posiciones}")

def main(args=None):
    rclpy.init(args=args)
    pincher = PincherController()
    Aplicacion(pincher)
    # No es necesario spin() para un movimiento puntual
    # rclpy.spin(node)  # habilita sólo si agregas callbacks

if __name__ == '__main__':
    main()