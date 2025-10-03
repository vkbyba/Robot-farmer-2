import socket, sys

IP, PORT = "192.168.0.105", 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

def send(cmd):
    sock.sendto((cmd+"\n").encode(), (IP, PORT))
    try:
        data, _ = sock.recvfrom(4096)
        print("RX:", data.decode(errors="ignore").strip())
    except socket.timeout:
        print("RX: <no reply>")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        send(" ".join(sys.argv[1:]))
    else:
        for c in ["PING","VER?","MAP?","HOME","S,A1,120","M,LEFT,120","M,RIGHT,120","STOP"]:
            print("TX:", c); send(c)
