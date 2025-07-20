import tkinter as tk, threading

class Aplicacion:
    """
    Clase que inicaliza y crea la ventana de inicio de sesion.
    """
    def __init__(self):
        # ----------CARACTERISTECAS DE VENTANA-------------------
        self.root = tk.Tk()
        #Obtener medidas de pantalla para centrar ventana.
        self.sw = self.root.winfo_screenwidth()
        self.sh = self.root.winfo_screenheight()
        self.x = self.sw // 3
        self.y = self.sh // 4
        self.root.title("RobotPincher")
        self.root.geometry(f"500x400+{self.x}+{self.y}")
        self.root.resizable(width=False, height=False)
        self.root.config(bg="white")

        #Se crea lienzo.
        self.Canva = tk.Canvas(self.root, width=self.sw, height=self.sh, bg='white')
        self.Canva.place(x=0, y=0)

        # ------------------BOTÓN POSICION 1--------------------

        self.bPos1 = tk.Button(self.root, text="Posicion 1", font=('Arial', 10), background='#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos1.place(x=80, y=70)

        # ------------------BOTÓN POSICION 2---------------------

        self.bPos2 = tk.Button(self.root, text="Posicion 2",font=('Arial', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos2.place(x=180, y=70)

        # -------------------BOTÓN POSICION 3------------------------
        self.bPos3 = tk.Button(self.root, text="Posicion 3",font=('Arial', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos3.place(x=280,y=70)
        
        # -------------------BOTÓN POSICION 4------------------------
        self.bPos4 = tk.Button(self.root, text="Posicion 4",font=('Arial', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos4.place(x=80, y=130)

        # -------------------BOTÓN POSICION 5------------------------
        self.bPos5 = tk.Button(self.root, text="Posicion 5",font=('Arial', 10), background= '#43046D',foreground='#FFFFFF', activebackground='red',command=self.Posicion1)
        self.bPos5.place(x=180,y=130)

        # -------------------BOTON HOME-----------------------------
        self.bHome = tk.Button(self.root, text="Ir HOME", font=('Arial', 10), background='#43046D', foreground='#FFFFFF', activebackground='red', command=self.Home)
        self.bHome.place(x=280, y= 130)

        # --------------MAINLOOP-------------------------
        self.root.mainloop()
    
    # ---------------METODOS ----------------------

    # ------ Posiciones --------------------------
    def Home(self):
        nueva_pos=[512, 512, 512, 512, 512]
        self.pincher.set_goal_positions(nueva_pos)
        self.Canva.create_text(200, 200, text='Posicion1', font=('Arial', 12), fill='Black', justify='center')

    def Posicion1(self):
        nuevas_pos = [0, 0, 0, 0, 0]
        self.pincher.set_goal_positions(nuevas_pos)

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

def main():
    """
    Funcion principal que abre la aplicacion.
    """
    Aplicacion()

#Condicional que revisa si se esta ejecutando desde el archivo se ha importado para poder ejecutar el main. 
if __name__ == "__main__":
    main()