import socket

# ESP32的IP地址和端口
#esp_ip = '192.168.82.185'
esp_ip = '192.168.187.185'
esp_port = 8080

arg = '''{"numActions": 1,"actionArray": ["Forw"],"durationArray": [500]}'''
    
s = socket.socket()
s.connect((esp_ip, esp_port))
# s.send(arg.encode()+ b'$')
s.send(b'2')
print("Command sent to ESP32")

# 设置超时时间，例如5秒
s.settimeout(2)

try:
    # 用来存储接收的原始数据
    data = b''

    # 循环接收数据，直到找到以'$'结尾的消息或超时
    while True:
        try:
            chunk = s.recv(1024) # 每次接收1024字节
            if chunk:
                data += chunk
                if data.endswith(b'$'):
                    # 如果数据以'$'结尾，表示消息接收完毕
                    break
            else:
                # 没有接收到数据，退出循环
                break
        except socket.timeout:
            # 超时
            print("Socket timeout")
            break
    print("Received from ESP32:", data.decode('utf-8')[:-1])

except UnicodeDecodeError as e:
    print("Decode error:", e)

finally:
    # 确保最终关闭socket
    s.close()
    print("Socket closed")