import socket #for receiving udp data
import struct #for making packets
from time import perf_counter #for time delay
from scipy.spatial.transform import Rotation #for converting Euler to quat for rotation

#delay for slowing down packet send rate. Currently unused
def accurate_delay(delay):
    time_goal = perf_counter() + delay/1000
    while perf_counter() < time_goal:
        pass
    

UDP_IP = "127.0.0.1" #local ip
UDP_PORT = 6969 #slimevr/owotrack port
packet_type = {"gyro":2, "handshake":3, "accel":4, "rotation":1} #hardcoded owotrack values to tell packet type

packet_id = 0 #counter for packets that gets incremented

gyro = [0, 0, 0] #example data
accel = [0, 0, 0] #example data
rotation = [0, 0, 0, 0] #example data

#default values to avoid errors
data_android = rotation 
data_ipod = accel
data_ipod.append(gyro) #appended as order goes accel then gyro in packets


#packets struct types, > is for endiness of data, most significant digit's placement
hs = struct.Struct (">i Q") #handshake
s = struct.Struct(">i Q 3f") #gyro and accelerometer
s2 = struct.Struct(">i Q 4f") #rotation
in_packet = struct.Struct("<2c 2c 3f 3f 3d 5d 2? 2i ? 2i") #for ipodtouch packets

#ipod server
receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver.bind(("0.0.0.0", 7070)) #0.0.0.0 receives data from any incoming connections, 7070 is port set in ipod app

#android magnetometer server
receiver2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
receiver2.bind(("0.0.0.0", 7071)) ##7071 is port set in android app

#local router for sending slimevr data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

#Send handshake
packet_data = (packet_type["handshake"], packet_id) 
packet = hs.pack(*packet_data)
sock.sendall(packet)

#Used to exit program properly when giving KeyboardInterrupt/hitting red square/stop button
receiver.settimeout(10.0)
receiver2.settimeout(10.0)
sock.settimeout(10.0)

#Received and Send data loop
try:
    while True:
        #Receive ipod touch data (gyro + accel) and unpack it from byes into array of data
        try:
            raw_data_ipod, address = receiver.recvfrom(111)
            data_ipod = in_packet.unpack(raw_data_ipod)
        except socket.timeout:
            print("ipod timeout")
        #print(f"GyroX-Y-Z: {data[7]:.2f}-{data[8]:.2f}-{data[9]:.2f}")
        #print(f"AcceX-Y-Z: {data[4]:.2f}-{data[5]:.2f}-{data[6]:.2f}")
        
        #Send Gyro data to SlimeVR Server (port 6969)
        packet_data = (packet_type["gyro"], packet_id, data_ipod[7], data_ipod[8], data_ipod[9]) 
        packet = s.pack(*packet_data)
        sock.sendall(packet)
        packet_id += 1
        
        
        #Send Accelerometer data to SlimeVR Server (port 6969)
        packet_data = (packet_type["accel"], packet_id, data_ipod[4], data_ipod[5], data_ipod[6]) 
        packet = s.pack(*packet_data)
        sock.sendall(packet)
        packet_id += 1
        
        
        #Receive android data (rotation)
        try:
            raw_data, address = receiver2.recvfrom(68)
            try:
                #Decode android data (rotiation) from bytes? -> string -> floats in an array
                data_android = [float(x) for x in (raw_data.decode(encoding='UTF-8',errors='strict').split(','))]
                
                #Convert Euler data to Quaternion
                android_rot = Rotation.from_euler('xyz', data_android, degrees=True)
                android_rot = android_rot.as_quat()
                #print(f"MagnetoXYZ: {data[0]:.2f}   {data[1]:.2f}   {data[2]:.2f}")
            except ValueError: #Android app sends empty data sometimes. This catches that error.
                pass
        except socket.timeout:
            print("android timeout")
        
        #Send android rotation data to SlimeVR Server (port 6969)
        packet_data = (packet_type["rotation"], packet_id, android_rot[0], android_rot[1], android_rot[2], android_rot[3]) 
        packet = s2.pack(*packet_data)
        sock.sendall(packet)
        packet_id += 1
        
        #accurate_delay(5)
        #print(f"Sending next packets:{packet_id}")
        #sleep(0.00001)

#Cleans up open ports so that kernal does not need to be restarted
except KeyboardInterrupt:
    receiver.shutdown
    receiver.close()
    sock.shutdown
    sock.close()
    receiver2.shutdown
    receiver2.close()
