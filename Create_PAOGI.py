import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import socket
import select
import re
#import utm
#import pynmea2
from datetime import datetime


update_rate = 100.0 # Hz
last_update = 0.0
last_print = 0.0

#IP and Port for the computer runnign AgOpenGPS
UDP_IP = '192.168.11.27'
UDP_PORT = 9999

#Internal port to set Emlid up.  Set Reachview output to server, localhost, port 5101
GPS_IP = '127.0.0.1'
GPS_PORT = 5101

BUFFER_SIZE = 1024

#create sockets
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
GPS_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
GPS_address = (GPS_IP, GPS_PORT)
gpsnum = 1
GPS_sock.setblocking(0)

#Initial configuration of messages and debugging
print_messages = True
running = True
gga_msg_on = False
rmc_msg_on = False
aog_msg_on = True
save_to_file = False
started_file = False
debugging = False

antennaHeight = 1.5 #Height from ground to antenna in METERS

#Allow user to enter an IP for the AgOpen Program as part of the start 
print "Default IP: ", UDP_IP 
new_ip = raw_input("Enter Receiving computer IP (blank for Default IP):")
if len(new_ip)>5:
    print new_ip
    UDP_IP=new_ip
print "UDP_IP is :", UDP_IP, "   Port: ", UDP_PORT

#Kalman  Variables
Pc = 0.0
G = 0.0
P = 1.0
Xp = 0.0
Zp = 0.0
KalRoll = 0.0
varRoll = 0.1
varProcess = .00001  ##smaller is more filtering
Kalman = True

#Connect to internal GPs message
try:
    print("connect")
    GPS_sock.close
    GPS_sock.connect((GPS_IP, GPS_PORT))
    GPS_Connection = True
    print("Connected to Reach")
except:
    print("Unable to connect to TCP socket for GPS data")
    GPS_Connection = False


print >> sys.stderr, 'conencting to', GPS_address
GPSsentence1 = "$GPGGA"
GPSsentence2 = "$GPRMC"
GPSsentence3 = "$GPVTG"
gps_qual_descriptors = ["invalid", "single","differential", "pps", "RTK fix", "RTK float", "Estimated", "Manual Input", "Simulation"]

#Read in the RTIMULib.ini file
SETTINGS_FILE = "RTIMULib"

print("Using settings file " + SETTINGS_FILE + ".ini")
if not os.path.exists(SETTINGS_FILE + ".ini"):
  print("Settings file does not exist, will be created")

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

print("IMU Name: " + imu.IMUName())

if (not imu.IMUInit()):
    print("IMU Init Failed")
    sys.exit(1)
else:
    print("IMU Init Succeeded")

# this is a good time to set any fusion parameters
# Slerp power controls the fusion and can be between 0 and 1
# 0 means that only gyros are used, 1 means that only accels/compass are used
# In-between gives the fusion mix.
imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)


poll_interval = imu.IMUGetPollInterval()
print("Recommended Poll Interval: %dmS\n" % poll_interval)
#print("Update Rate:" ,update_rate, " %6.2f" %(1/update_rate))

print ("Configuring....")
time.sleep(1)
now = time.time()
GPS_data = ''

#Print out initial IMU reads before starting the looping.  confirms IMU is working
i=0
while i<100:
    if imu.IMURead():
        print("i", i, "data", imu.getIMUData())
    i+=1

#Function to take the string and break it into individual RMC and GGA NMEA sentences.
    #Note: VTG is mentioned but not used in the program
def read_GPS():
    try:
        stringdata1 = GPS_sock.recv(BUFFER_SIZE)
        #print(decode_NMEA(stringdata1))
        GGA, RMC = decode_NMEA(stringdata1)
        if (GGA != 0 and RMC != 0):
            #print "intime : % 15.3f" %( time.time())
            #h, m, s = str(RMC.timestamp).split(':')
            #GPS_seconds=int(h)*3600+int(m)*60+float(s)
            GPS_Update = True
        else:
            GPS_Update=False
    except: #socket.error, v:
        GPS_Update = False
        GGA = None
        RMC = None
        VTG = None
    return(GGA, RMC, GPS_Update)
    
#Function to split comma separated NMEA sentence into a list of the individual data fields
def get_csv_chunks(CSV, chunks):
    split_csv = CSV.split(',')
    return_list=[]
    for fields in chunks:
        return_list.append(split_csv[fields])
    return (return_list)


def decode_NMEA(stringdata1):
    RMC_update = False
    GGA_update = False
    VTG_update = False
    for lines in stringdata1.splitlines():
        if GPSsentence1 in lines:
            #print("Found 1:", GPSsentence1, lines)
            try: 
                #Confirm teh sentence has a valid checksum
                validGGA = checksum_nmea(lines)
                if(validGGA == True):
                    
                    GGA=lines
                    GGA_update=True
                    
            except:
                print("Invalid gga.  GPS # : ", gpsnum)
                GGA_update = False
            
        elif GPSsentence2 in lines:
            #print("Found 2:", GPSsentence2, lines)
            try: 
                validRMC = checksum_nmea(lines)
                if (validRMC==True):
                    RMC = lines
                    RMC_update = True
                    
            except:
                RMC_update = False
            
            
        elif GPSsentence3 in lines:
            #print("Found 2:", GPSsentence2, lines)
            try: 
                validVTG = checksum_nmea(lines)
                
                if (validVTG==True):
                    VTG = lines
                    VTG_update = True
                    
            except:
                VTG_update = False
            
    if(RMC_update == True or GGA_update == True):
        return(GGA, RMC)
    else:
        return(0,0)
    
#Function to compute and validate checksum of a sentence.
# Requires import re
def checksum_nmea(sentence):
    cksum = sentence[len(sentence) -2:]
    #Strip new line and find all characters between $ and *
    chksumdata = re.sub("(\n|\r\n)", "", sentence[sentence.find("$")+1:sentence.find("*")])
    csum = 0
    for c in chksumdata:
        csum ^= ord(c)
    if hex(csum) == hex(int(cksum,16)):
        return(True)
        
    else:
        print("Bad Checksum data, Returning False: ", sentence)
        return(False)

#Stub of code to do roll and pitch correction directly on Reach.  This is NOT WORKING AND CURRENT NOT UNDER DEVELOPMENT
##def correct_for_angles(roll, pitch, RMC, heading):  # roll and pitch should be in RADIANS
##    rollCorrectionDistance = antennaHeight* math.tan(roll)
##    pitchCorrectionDistance = antennaHeight*math.tan(pitch)
##    #print "Roll corr: % 5.2f  Pitch corr: % 5.2f" % (rollCorrectionDistance, pitchCorrectionDistance)
##    headingRad = math.radians(heading)
##    ## need to figure out sign convention and make sure it's right!!
##    utmdata=utm.from_latlon(RMC.latitude, RMC.longitude)
##    
##    #Correct for Roll
##    ## UNVALIDATED!!!!! Need to go through and cross check these!!!!!
##    "The math and signs here need checked and validated!!!"
##    correctedEasting = utmdata[0]+ (rollCorrectionDistance * math.cos(headingRad))
##    correctedNorthing = utmdata[1]+(rollCorrectionDistance * (math.sin(headingRad)))
##    #Correct for Pitch
##    correctedEasting = correctedEasting + (pitchCorrectionDistance * math.sin(headingRad))
##    correctedNorthing = correctedNorthing + (pitchCorrectionDistance * (math.cos(headingRad)))
##    #print correctedEasting-utmdata[0], correctedNorthing-utmdata[1]
##    
##    correctedLatLon = utm.to_latlon(correctedEasting, correctedNorthing, utmdata[2], utmdata[3])
##    
##    return (correctedLatLon)
##
##def output_NMEA(rmc, gga, CorrectedLatLon):
##    gga_time_out = "%02d" % (gga.timestamp.hour) + "%02d" % (gga.timestamp.minute)+"%02d" % (gga.timestamp.second)+"."+ "%02d" % ((gga.timestamp.microsecond)/10000)
##    rmc_time_out = "%02d" % (rmc.timestamp.hour) + "%02d" % (rmc.timestamp.minute)+"%02d" % (rmc.timestamp.second)+"."+ "%02d" % ((rmc.timestamp.microsecond)/10000)
##    datestr = datetime.strftime(datetime.utcnow(), '%m%d%y')
##    CorrectedLatLonStr = create_LatLon_String(CorrectedLatLon)
##    NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (gga_time_out, CorrectedLatLonStr[0], gga.lat_dir, CorrectedLatLonStr[1], gga.lon_dir, str(gga.gps_qual), gga.num_sats, gga.horizontal_dil, str(gga.altitude), gga.altitude_units, gga.geo_sep, gga.geo_sep_units, gga.age_gps_data, gga.ref_station_id))
##    NMEAoutRMC = pynmea2.RMC('GP','RMC', (rmc_time_out, 'A', CorrectedLatLonStr[0], rmc.lat_dir, CorrectedLatLonStr[1], rmc.lon_dir, str(round(rmc.spd_over_grnd,2)), str(round(rmc.true_course,2)), datestr, rmc.mag_variation))
##    #print NMEAoutGGA
##    #print NMEAoutRMC
##    return(NMEAoutRMC, NMEAoutGGA)
##"Next step is to write teh NMEA strings to a TCP port for AgOpen to read."
##
##def create_LatLon_String(LatLon):
##    Latdeg = int(LatLon[0])
##    Latmin = int((LatLon[0]-Latdeg)*60)
##    Latsec = ((LatLon[0]-Latdeg)*60)-Latmin
##    Latsec = round(Latsec,7)
##    LatStrng = "%02d" % (Latdeg)+"%02d" %(Latmin)+ (str(Latsec).replace('0', '', 1))
##    Lon = abs(LatLon[1])
##    Londeg = (int(Lon))
##    Lonmin = (int((Lon-Londeg)*60))
##    Lonsec = (((Lon-Londeg)*60)-Lonmin)
##    Lonsec = round(Lonsec,7)
##    LonStrng = "%03d" % (Londeg)+"%02d" %(Lonmin)+ str(Lonsec).replace('0', '', 1)
##    return(LatStrng, LonStrng)

#Combine the info from GGA, RMC and IMU to build the Paogi sentence
def build_PAOGI(gga, rmc, roll, pitch, yaw, gyro, IMUStatus):
    csum = 0
    if(yaw < 0):
       heading = yaw+360
    if (yaw >= 0):
        heading = yaw
    yawrate = round(math.degrees(gyro[2]), 2)
    IMUStatus = str(IMUStatus)[:1]
    #This selected the individual data fields in the gga and rmc sentences that are needed
    gga_chunk = get_csv_chunks(gga, [1,2,3,4,5,6,7,8,9,10,13])
    rmc_chunk = get_csv_chunks(rmc, [7, 8])

    PAOGI = "PAOGI,"
    for items in gga_chunk:
        PAOGI+=items+","
    for items in rmc_chunk:
        PAOGI+=items+","
    PAOGI = PAOGI+str(-1*roll)+","+str(pitch)+","+str(heading)+","+str(yawrate)+","+IMUStatus
    #PAOGI = PAOGI+str(round(math.degrees(fusionPose[0]),2))+","+str(round(math.degrees(fusionPose[1]),2))+","+str(heading)+","+str(yawrate)+","+IMUStatus+","
    
#compute and append checksum
    for c in PAOGI:
        csum ^= ord(c)
    csum=str(hex(csum))[-2:]
    csum= csum.upper()
    PAOGI = "$"+PAOGI+"*"+csum
    
    return(PAOGI)
    

if __name__ == "__main__":
    try:
        
        while running:
            
            if imu.IMURead():
                #print("In IMU read")
                now=time.time()
                data = imu.getIMUData()
                #print(data)
                fusionPose = data["fusionPose"]
                IMUStatus = data["fusionPoseValid"]
                gyro = data["gyro"]
                #yaw = math.degrees(fusionPose[2])-90  ## this is in to match BNO on my mounting box.
                yaw = round(math.degrees(fusionPose[2]),2)
                if(yaw < 0):
                   heading = yaw+360
                if (yaw >= 0):
                    heading = yaw
                roll = math.degrees(fusionPose[0])
                pitch = math.degrees(fusionPose[1])
                if(Kalman == True):
                    Pc = P + varProcess
                    G = Pc / (Pc+varRoll)
                    P = (1-G)*Pc
                    Xp = KalRoll
                    Zp = Xp
                    KalRoll = (G*(roll-Zp))+Xp
                    
                GGA, RMC, GPS_Update=read_GPS()
                if GPS_Update:
                    last_update = now
                    current_GGA = GGA
                    current_RMC = RMC
                                        
                    PAOGI = build_PAOGI(current_GGA, current_RMC, KalRoll, pitch, yaw, gyro, IMUStatus) 
                    
                    message = PAOGI+'\r\n'
                    
                
                    if gga_msg_on:
                        message= message+current_GGA+'\r\n'
                    if rmc_msg_on:
                        message = message+current_RMC+'\r\n'
                    sock.sendto(message, (UDP_IP, UDP_PORT))
                    if print_messages:
                        print "Sent Message:"
                        print message
                    output_string = message
                    if debugging:
                        print "Roll:", roll, " KalRoll:", KalRoll
                   
                    last_print = now
                    
                    if (save_to_file == True and started_file == True):
                        with open(file_name, 'a') as text_file:
                            text_file.write (output_string[:-4]+ str(roll)+"\r\n")

#Bit of code to monitor keyboard input (works over SSH) and set variable based on what has been typed in
                            #Just type in teh codes and press enter while the system running.  You want' be able to easily see
                            #what is types in as teh messages will keep running over teh keyboard input.
                            
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                keystroke = sys.stdin.readline().rstrip()
                print "You entered", keystroke
                if keystroke == "m":
                    print_messages = not print_messages
                    print "Display messages on screen set to:", print_messages
                elif keystroke == "X":
                    print "Exiting Program"
                    GPS_sock.close()
                    sock.close()
                    running = False
                    break
                elif keystroke == 'gga':
                    gga_msg_on=not gga_msg_on
                    print "gga messages are set to:", gga_msg_on
                elif keystroke == 'rmc':
                    rmc_msg_on=not rmc_msg_on
                    print "rmc messages are set to:", rmc_msg_on
                elif (keystroke == 's'):
                    if started_file == False:
                        now=datetime.now()
                        started_file = True
                        file_name = ("./datafiles/data_"+str(now.month)+ "_"+str(now.day)+"_"+str(now.hour)+"_"+str(now.minute)+".csv")
                        with open(file_name, 'w') as text_file:
                            text_file.write("PAOGI, UTCseconds, Lat, LatDir, Lon, LonDir, FixQual, Sats, HDOP, Alt, AltUnits, DGPS Age, Spd_KPH, Hdg_True, IMU_HDG_True, Roll_Kal, Pitch, Yaw_rate, IMU_Status, Roll"+ '\n')
                    save_to_file = not save_to_file
                    print "Save to file is set to: ", save_to_file
                elif (keystroke == 'debug'):
                    debugging = not debugging
                elif (keystroke.split("=")[0] == 'varp'):
                    varProcess = float(keystroke.split("=")[1])

                else:
                    print "you entered: ", keystroke, "  No action being taken"
#Brief sleep timer to reduce CPU load.  Runs at approximately 100 Hz
            time.sleep(0.009)
            
    except KeyboardInterrupt:
        print("interrupted!")
        GPS_sock.close()
        sock.close()
    
print (data)
      
