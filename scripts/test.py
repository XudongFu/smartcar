
import socket
import math
print(math.degrees(1.3962634))


sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sk.bind(("192.168.199.243",1234))
sk.listen(5)
client = sk.accept()
print("accepted")