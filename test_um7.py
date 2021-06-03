import um7
import time

phithetapsi = [0,0,0]

#----------initialize um7 Serial---------------------
try:
	um7 = um7.UM7(serial_port='/dev/ttyS0', baudrate=115200) # rpi3 serial port is /dev/ttyS0
except:
	print("um7 initialize failed.")
	print("um7 not plugged in or failed.")

#----------------read values and display-----------------
while True:
    phithetapsi = um7.rollpitchyaw()
    roll = phithetapsi[0]
    pitch = phithetapsi[1]
    heading = phithetapsi[2]
    #---- heading correction for negative values----------------
    if heading < 0:
        heading = round(heading + 360,3)

    print(f"roll: {roll}, pitch: {pitch}, heading: {heading}")
    
    time.sleep(0.5)