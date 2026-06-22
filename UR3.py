import socket
import time

# Dirección IP del robot UR
HOST = "10.10.73.235"

# Puerto del servidor en el robot
PORT = 30002

# Scripts para abrir y cerrar la pinza
Abrir_pinza = 'pinza40UR3.py'
Cerrar_pinza = 'pinza10UR3.py'

PI=3.141592653589793


def g2r(n):
    for i in range(len(n)):
        n[i]*=PI/180
    return n

# Función para enviar una trayectoria en espacio de configuraciones a la controladora del robot
def send_joint_path(path, sock):
    for joint_config in path:
        print(joint_config)
        sock.send(f"movej({joint_config}, a=0.5, v=0.5)".encode() + "\n".encode())
        time.sleep(3)

# Conexión via socket a la controladora del robot
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

print("connectat")

# Trayectoria -- configuraciones (variables articulares, en radianes)
path_primer_bloque = [
    [0.3923500158483253, -1.873261886165514, -1.2201596800692358, -1.544441855089782, -4.738394386239405, 0.31695179216217023],
    [0.393048147549123, -1.8978710286186338, -1.479515606915593, -1.2606513187155044, -4.7380453203890065, 0.317649923862968]

    # baja a por la segunda, sube, la deja, sube. 
]
path_dejar_bloque_1 = [   
    [0.3923500158483253, -1.873261886165514, -1.2201596800692358, -1.544441855089782, -4.738394386239405, 0.31695179216217023],
    [-0.04537856055185257, -2.1889919478512883, -1.0861183935160712, -1.3583897568271868, -4.704185932900316, 6.16223399001638]
]

path_buscar_segundo_bloque = [
    [-0.040491638646268445, -2.1963223307096644, -0.6218608124855797, -1.8161896196252993, -4.704884064601114, 6.168168109473161],
    [0.3923500158483253, -1.873261886165514, -1.2201596800692358, -1.544441855089782, -4.738394386239405, 0.31695179216217023],
    [0.3883357585687383, -1.9202112430441614, -1.508837138349098, -1.2088150399312727, -4.737521721613408, 0.3129375348825833]
]

path_dejar_segundo_bloque = [
    [0.3923500158483253, -1.873261886165514, -1.2201596800692358, -1.544441855089782, -4.738394386239405, 0.31695179216217023],
    [-0.22689280275926285, -2.319542575900464, -0.8861036612375212, -1.4189526818713898, -4.674864401466812, 6.058910498298315]
]
    # Se envia la trayectoria a la controladora del robot
send_joint_path(path_primer_bloque, sock)
with open(Cerrar_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
send_joint_path(path_dejar_bloque_1, sock)
with open(Abrir_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
send_joint_path(path_buscar_segundo_bloque, sock)
with open(Cerrar_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
send_joint_path(path_dejar_segundo_bloque, sock)
# Enviar archivo script cerrar pinza
with open(Abrir_pinza, 'rb') as f: sock.sendall(f.read())
time.sleep(1)
send_joint_path(path_primer_bloque, sock)


# Se envia la trayectoria a la controladora del robot

print("Trayectoria finalizada")

data = sock.recv(1024)

# Se cierra la conexión
sock.close()
