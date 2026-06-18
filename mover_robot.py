import socket
import time

# Dirección IP del robot UR
HOST = "10.10.73.23X"

# Puerto del servidor en el robot
PORT = 30002

# Scripts para abrir y cerrar la pinza
Abrir_pinza = 'pinza40UR3.py'
Cerrar_pinza = 'pinza10UR3.py'

# Función para enviar una trayectoria en espacio de configuraciones a la controladora del robot
def send_joint_path(path, sock):
    for joint_config in path:
        print(joint_config)
        sock.send(f"movej({joint_config}, a=0.5, v=0.5)".encode() + "\n".encode())
        time.sleep(1)

# Conexión via socket a la controladora del robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Trayectoria -- configuraciones (variables articulares, en radianes)
path = [
    [-0.4252,-2.3921,0.5604,-1.3436,1.6586,0.0059],
    [0.2353,-2.3921,0.5604,-1.3436,1.6586,0.0059],
    # Añadir más si hiciera falta
]

# Se envia la trayectoria a la controladora del robot
send_joint_path(path, sock)
# Enviar archivo script abrir pinza
with open(Cerrar_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
# Se envia la trayectoria a la controladora del robot
send_joint_path(path, sock)
# Enviar archivo script cerrar pinza
with open(Abrir_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
# Se envia la trayectoria a la controladora del robot
send_joint_path(path, sock)

# Mensaje que se imprime cuando se finaliza la ejecución
# de la trayectoria
print("Trayectoria finalizada")

data = sock.recv(1024)

# Se cierra la conexión
sock.close()