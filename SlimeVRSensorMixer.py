import socket #for receiving udp data
import struct #for making packets
from time import perf_counter #for time delay
from scipy.spatial.transform import Rotation #for converting Euler to quat for rotation
from math import sqrt

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

min_max = [[180, 0], [180, 0], [180, 0]]

#default values to avoid errors
data_android = rotation 
data_ipod = accel
data_ipod.append(gyro) #appended as order goes accel then gyro in packets
android_rot_quat = rotation
rotate_vector = [0, 0, 0]

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
                
                #min_max calc
                # if (data_android[0] < min_max[0][0]):
                #     min_max[0][0] = data_android[0]
                # if (data_android[0] > min_max[0][1]):
                #     min_max[0][1] = data_android[0]
                    
                # if (data_android[1] < min_max[1][0]):
                #     min_max[1][0] = data_android[1]
                # if (data_android[1] > min_max[1][1]):
                #     min_max[1][1] = data_android[1]
                
                # if (data_android[2] < min_max[2][0]):
                #     min_max[2][0] = data_android[2]
                # if (data_android[2] > min_max[2][1]):
                #     min_max[2][1] = data_android[2]
                
                #Normalize android data
                #omega_magnitude = sqrt(data_android[0] * data_android[0] + data_android[1] * data_android[1] + data_android[2] * data_android[2])
                #data_android[0] /= omega_magnitude
                #data_android[1] /= omega_magnitude
                #data_android[2] /= omega_magnitude
                
                #temp1 = raw_data.decode(encoding='UTF-8', errors='strict')
                #print(f"Decoded:{temp1}")
                
                #temp2 = temp1.split(',')
                #print(f"Split:{temp2}")
                 
                #data_android = [float(x) for x in temp2]
                #print(f"Floats:{data_android}")
                
                #covert to degrees for Rotation functions
                data_android_degrees = []
                for val in data_android:
                    data_android_degrees.append(180 * val)
                    
                #print(f"Degrees:{data_android_degrees}")
                
                
                #Convert Euler data to Quaternion
                #XYZ not xyz as the rotations are intrinsic (sensors move with device) not extrinsic (global reference point), I think?
                android_rot_euler = Rotation.from_euler('XYZ', data_android_degrees, degrees=True) 
                #print(f"Before Rotation: {android_rot_euler.as_matrix()}")
                #print(f"From Normal: {Rotation.from_euler('XYZ', data_android, degrees=False).as_matrix()}")
                #android_rot_euler_rotated = Rotation.from_euler(android_rot_euler.apply(rotate_vector))
                #print(android_rot_euler.apply(rotate_vector))
                #temp1 = android_rot_euler.apply(rotate_vector)
                #temp2 = Rotation.from_rotvec(temp1)
                #temp3 = temp2.as_euler("XYZ")
                #print(f"After Rotation: {temp3.as_matrix()}")
                
                
                android_rot_quat = android_rot_euler.as_quat()
                
                #android_rot_vec = Rotation.from_rotvec(data_android)
                #android_rot_quat = android_rot_vec.as_quat()
                
                # print(f"Rotation: {data_android[0]:.3f}   {data_android[1]:.3f}   {data_android[2]:.3f}")
            except ValueError: #Android app sends empty data sometimes. This catches that error.
                print("ValueError")
                pass
        
        except socket.timeout:
            print("android timeout")
        
        #Send android rotation data to SlimeVR Server (port 6969)
        #try:
        #multiplying the y (android_rot_quat[1]) here by -1 to flip it seems to make the game rotation sensor somewhat accurate
        packet_data = (packet_type["rotation"], packet_id, android_rot_quat[0], (android_rot_quat[1]), android_rot_quat[2], android_rot_quat[3]) 
        packet = s2.pack(*packet_data)
        sock.sendall(packet)
        packet_id += 1
        
        #print(f"data_android:{data_android[0]:.3f}|{data_android[1]:.3f}|{data_android[2]:.3f}")
        #print(f"euler:{android_rot_euler}")
        #print(f"rot_vec:{android_rot_vec}")
        #print(f"quat:{android_rot_quat[0]:.3f}|{android_rot_quat[1]:.3f}|{android_rot_quat[2]:.3f}|{android_rot_quat[3]:.3f}")
        #except NameError:
        #    print("Rotation data skipped: NameError")
        #    pass
        
        #accurate_delay(5)
        #print(f"Sending next packets:{packet_id}")
        #sleep(0.00001)

#Cleans up open ports so that kernal does not need to be restarted
except KeyboardInterrupt:
    pass

finally:
    # print(f"min_max: {min_max}")
    receiver.shutdown
    receiver.close()
    sock.shutdown
    sock.close()
    receiver2.shutdown
    receiver2.close()
