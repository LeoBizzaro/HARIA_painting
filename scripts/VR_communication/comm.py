import socket
import rospy
from geometry_msgs.msg import PoseStamped

# === PARAMETRI TCP ===
SERVER_IP = "192.168.1.100"   # <--- Inserisci l'IP del server
SERVER_PORT = 5005            # <--- Inserisci la porta del server

# === ROS SETUP ===
rospy.init_node('tcp_pose_client_2d')
pub = rospy.Publisher('/demo/pose_final', PoseStamped, queue_size=10)
rate = rospy.Rate(100)

# === TCP SETUP ===
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    rospy.loginfo(f"Connessione a {SERVER_IP}:{SERVER_PORT}...")
    sock.connect((SERVER_IP, SERVER_PORT))
    rospy.loginfo("Connesso al server.")

    buffer = b""
    while not rospy.is_shutdown():
        data = sock.recv(1024)
        if not data:
            rospy.logwarn("Connessione interrotta dal server.")
            break

        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            try:
                # Parsing manuale della stringa (x,y)
                text = line.decode().strip()
                if text.startswith("(") and text.endswith(")"):
                    text = text[1:-1]  # rimuove le parentesi
                    x_str, y_str = text.split(",")
                    x = float(x_str.strip())
                    y = float(y_str.strip())

                    # Crea e pubblica il messaggio PoseStamped
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.pose.position.x = x
                    pose_msg.pose.position.y = y
                    pose_msg.pose.position.z = 0.0

                    # Orientamento nullo
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    pose_msg.pose.orientation.w = 1.0

                    pub.publish(pose_msg)
                else:
                    rospy.logwarn(f"Formato messaggio non valido: {text}")

            except Exception as e:
                rospy.logerr(f"Errore nel parsing della linea: {e}")

        rate.sleep()

except Exception as e:
    rospy.logerr(f"Errore TCP: {e}")
finally:
    sock.close()
    rospy.loginfo("Socket chiuso.")
