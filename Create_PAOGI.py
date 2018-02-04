import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import time
import math
import socket
import select
import re
import utm
import pynmea2
from datetime import datetime


update_rate = 100.0 # Hz
last_update = 0.0
last_print = 0.0
UDP_IP = '192.168.11.27'
UDP_PORT = 9999
GPS_IP = '127.0.0.1'
GPS_PORT = 5101
BUFFER_SIZE = 1024
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
GPS_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
GPS_address = (GPS_IP, GPS_PORT)
gpsnum = 1
GPS_sock.setblocking(0)
print_messages = True
running = True
gga_msg_on = False
rmc_msg_on = False
aog_msg_on = True
save_to_file = False
started_file = False
debugging = False

antennaHeight = 1.5 #Height from ground to antenna in METERS
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


try:
    print("connect")
    GPS_sock.close
    GPS_sock.connect((GPS_IP, GPS_PORT))
    GPS_Connection = True
    print("Connected to Reach 2")
except:
    print("Unable to connect to TCP socket for GPS2 data")
    GPS_Connection = False


print >> sys.stderr, 'conencting to', GPS_address
GPSsentence1 = "$GPGGA"
GPSsentence2 = "$GPRMC"
GPSsentence3 = "$GPVTG"
gps_qual_descriptors = ["invalid", "single","differential", "pps", "RTK fix", "RTK float", "Estimated", "Manual Input", "Simulation"]

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
time.sleep(2)
now = time.time()
GPS_data = ''
i=0
while i<100:
    if imu.IMURead():
        print("i", i, "data", imu.getIMUData())
    i+=1
    
def read_GPS():
    try:
        stringdata1 = GPS_sock.recv(BUFFER_SIZE)
        #print(stringdata1, file=text_file)
        #print("Stringdata", stringdata1)
        #NMEA = read_data(s2, s2_info, 2)
        #print(decode_NMEA(stringdata1))
        GGA, RMC = decode_NMEA(stringdata1)
        #print("GGA", GGA, "  RMC", RMC)
        if (GGA != 0 and RMC != 0):
            #print "intime : % 15.3f" %( time.time())
            #h, m, s = str(RMC.timestamp).split(':')
            #GPS_seconds=int(h)*3600+int(m)*60+float(s)
            GPS_Update = True
            #print("GPS Update is true")
            #time.print(GPS_seconds)
        else:
            GPS_Update=False
    except: #socket.error, v:
        GPS_Update = False
        GGA = None
        RMC = None
        VTG = None
                #print ("GPS read error")
    #print("GPS_UPDATE", GPS_Update)
    return(GGA, RMC, GPS_Update)
    ####          GPS_Update = False
    ####          print("False GPs Update")
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
        #print("Lines:", lines)
        if GPSsentence1 in lines:
            #print("Found 1:", GPSsentence1, lines)
            try: 
                #print("going into checksum")
                validGGA = checksum_nmea(lines)
                #print("Valid GGA", validGGA)
                if(validGGA == True):
                    #print("starting Parse, GGA")
                    #GGA=pynmea2.parse(lines)  # parse the GPGGA string
                    GGA=lines
                    #print("GGA after Parse")
                    GGA_update=True
                    #print(GGA)
                    #q.put(GGA)
                    #print(gps_qual_descriptors[GGA.gps_qual], end="")
                    
            except:
                print("Invalid gga.  GPS # : ", gpsnum)
                GGA_update = False
            
        elif GPSsentence2 in lines:
            #print("Found 2:", GPSsentence2, lines)
            try: 
                validRMC = checksum_nmea(lines)
                #print(validRMC)
                if (validRMC==True):
                    #RMC=pynmea2.parse(lines)  # parse the string
                    RMC = lines
                    RMC_update = True
                    #print("lines", lines)
                    #print("RMC", RMC)
                    #qRMC.put_nowait(RMC)
                    #print("New RMC", gpsnum, RMC)
                    #print(", {:%H:%M:%S.%f}".format(RMC.timestamp), end="")
                    #print(", Lat: {:.8f}".format(RMC.latitude), end="")
                    #print(", Lon: {:.8f}".format(RMC.longitude), end="")
                    #print(", Hdg: {:.2f}".format(RMC.true_course), end="")
                    #print(", Spd: {:.2f}".format(RMC.spd_over_grnd))
                    
            except:
                RMC_update = False
            
            
        elif GPSsentence3 in lines:
            #print("Found 2:", GPSsentence2, lines)
            try: 
                validVTG = checksum_nmea(lines)
                #print(validRMC)
                if (validVTG==True):
                    #RMC=pynmea2.parse(lines)  # parse the string
                    VTG = lines
                    VTG_update = True
                    #print("lines", lines)
                    #print("RMC", RMC)
                    #qRMC.put_nowait(RMC)
                    #print("New RMC", gpsnum, RMC)
                    #print(", {:%H:%M:%S.%f}".format(RMC.timestamp), end="")
                    #print(", Lat: {:.8f}".format(RMC.latitude), end="")
                    #print(", Lon: {:.8f}".format(RMC.longitude), end="")
                    #print(", Hdg: {:.2f}".format(RMC.true_course), end="")
                    #print(", Spd: {:.2f}".format(RMC.spd_over_grnd))
                    
            except:
                VTG_update = False
            
            
    #print("decodeNMEAcomplete", gps_qual_descriptors[GGA.gps_qual])
##    if(len(str(GGA.gps_qual))>0):
##        #print(gps_qual_descriptors[GGA.gps_qual], end="")
##        
##    if (RMC.true_course>0):
##        print(", {:%H:%M:%S.%f}".format(RMC.timestamp), end="")
##        print(", Lat: {:.8f}".format(RMC.latitude), end="")
##        print(", Lon: {:.8f}".format(RMC.longitude), end="")
##        print(", Hdg: {:.2f}".format(RMC.true_course), end="")
##        print(", Spd: {:.2f}".format(RMC.spd_over_grnd))
    if(RMC_update == True or GGA_update == True):
        return(GGA, RMC)
    else:
        return(0,0)
    

def checksum_nmea(sentence):
    ## Required import re
    cksum = sentence[len(sentence) -2:]
    #print("In checksum")
    chksumdata = re.sub("(\n|\r\n)", "", sentence[sentence.find("$")+1:sentence.find("*")])
    #print ("chksumdata",chksumdata)
    csum = 0
    for c in chksumdata:
        csum ^= ord(c)
    #vprint("Checksum:", hex(csum), "Target Chksum:", hex(int(cksum,16)))
    if hex(csum) == hex(int(cksum,16)):
        #print("Valid Checksum, returrning True")
        return(True)
        
    else:
        print("Bad Checksum data, Returning False")
        print sentence
        return(False)
      
def correct_for_angles(roll, pitch, RMC, heading):  # roll and pitch should be in RADIANS
    rollCorrectionDistance = antennaHeight* math.tan(roll)
    pitchCorrectionDistance = antennaHeight*math.tan(pitch)
    #print "Roll corr: % 5.2f  Pitch corr: % 5.2f" % (rollCorrectionDistance, pitchCorrectionDistance)
    headingRad = math.radians(heading)
    ## need to figure out sign convention and make sure it's right!!
    utmdata=utm.from_latlon(RMC.latitude, RMC.longitude)
    
    #Correct for Roll
    ## UNVALIDATED!!!!! Need to go through and cross check these!!!!!
    "The math and signs here need checked and validated!!!"
    correctedEasting = utmdata[0]+ (rollCorrectionDistance * math.cos(headingRad))
    correctedNorthing = utmdata[1]+(rollCorrectionDistance * (math.sin(headingRad)))
    #Correct for Pitch
    correctedEasting = correctedEasting + (pitchCorrectionDistance * math.sin(headingRad))
    correctedNorthing = correctedNorthing + (pitchCorrectionDistance * (math.cos(headingRad)))
    #print correctedEasting-utmdata[0], correctedNorthing-utmdata[1]
    
    correctedLatLon = utm.to_latlon(correctedEasting, correctedNorthing, utmdata[2], utmdata[3])
    
    return (correctedLatLon)

def output_NMEA(rmc, gga, CorrectedLatLon):
    gga_time_out = "%02d" % (gga.timestamp.hour) + "%02d" % (gga.timestamp.minute)+"%02d" % (gga.timestamp.second)+"."+ "%02d" % ((gga.timestamp.microsecond)/10000)
    rmc_time_out = "%02d" % (rmc.timestamp.hour) + "%02d" % (rmc.timestamp.minute)+"%02d" % (rmc.timestamp.second)+"."+ "%02d" % ((rmc.timestamp.microsecond)/10000)
    datestr = datetime.strftime(datetime.utcnow(), '%m%d%y')
    CorrectedLatLonStr = create_LatLon_String(CorrectedLatLon)
    NMEAoutGGA = pynmea2.GGA('GP', 'GGA', (gga_time_out, CorrectedLatLonStr[0], gga.lat_dir, CorrectedLatLonStr[1], gga.lon_dir, str(gga.gps_qual), gga.num_sats, gga.horizontal_dil, str(gga.altitude), gga.altitude_units, gga.geo_sep, gga.geo_sep_units, gga.age_gps_data, gga.ref_station_id))
    NMEAoutRMC = pynmea2.RMC('GP','RMC', (rmc_time_out, 'A', CorrectedLatLonStr[0], rmc.lat_dir, CorrectedLatLonStr[1], rmc.lon_dir, str(round(rmc.spd_over_grnd,2)), str(round(rmc.true_course,2)), datestr, rmc.mag_variation))
    #print NMEAoutGGA
    #print NMEAoutRMC
    return(NMEAoutRMC, NMEAoutGGA)
"Next step is to write teh NMEA strings to a TCP port for AgOpen to read."

def create_LatLon_String(LatLon):
    Latdeg = int(LatLon[0])
    Latmin = int((LatLon[0]-Latdeg)*60)
    Latsec = ((LatLon[0]-Latdeg)*60)-Latmin
    Latsec = round(Latsec,7)
    LatStrng = "%02d" % (Latdeg)+"%02d" %(Latmin)+ (str(Latsec).replace('0', '', 1))
    Lon = abs(LatLon[1])
    Londeg = (int(Lon))
    Lonmin = (int((Lon-Londeg)*60))
    Lonsec = (((Lon-Londeg)*60)-Lonmin)
    Lonsec = round(Lonsec,7)
    LonStrng = "%03d" % (Londeg)+"%02d" %(Lonmin)+ str(Lonsec).replace('0', '', 1)
    return(LatStrng, LonStrng)

def build_PAOGI(gga, rmc, roll, pitch, yaw, gyro, IMUStatus):
##    $PAOGI (Propreitary, AgOpenGPS, Imu
##Timestamp
##Heading xxx.xx (Degrees)
##Magnetic/True Flag - M / T 
##Roll xx.xx (Degrees)
##Pitch xx.xx (degrees)
##Yaw Rate xxx.x (deg / second) - future thoughts here for steering
##PoseValid T/F 
##Speed xx.xx (MPH?) - future thoughts here if too slow for GPS speed
    csum = 0
    if(yaw < 0):
       heading = yaw+360
    if (yaw >= 0):
        heading = yaw
    yawrate = round(math.degrees(gyro[2]), 2)
    IMUStatus = str(IMUStatus)[:1]
    gga_chunk = get_csv_chunks(gga, [1,2,3,4,5,6,7,8,9,10,13])
    rmc_chunk = get_csv_chunks(rmc, [7, 8])
    #rmc_chunk[0] = str(float(rmc_chunk[0])*1.852)  #Converts knots to KPH if needed
    
    
    #print IMUStatus, "  ", yawrate
    #PAOGI = "PAOGI"+","+timestamp+","+str(heading)+","+"T"+","+str(round(math.degrees(fusionPose[0]),2))+","+str(round(math.degrees(fusionPose[1]),2))+","+str(yawrate)+","+IMUStatus+","+str(99.99)
    
    #$GAOGI,191939.00,5326.3450216,N,11109.60282,W,3,7,0.9,20.09876,M,1.2,4.9,0,0.11,0.12,359.9,T,*7A
    PAOGI = "PAOGI,"
    for items in gga_chunk:
        PAOGI+=items+","
    for items in rmc_chunk:
        PAOGI+=items+","
    PAOGI = PAOGI+str(-1*roll)+","+str(pitch)+","+str(heading)+","+str(yawrate)+","+IMUStatus
    #PAOGI = PAOGI+str(round(math.degrees(fusionPose[0]),2))+","+str(round(math.degrees(fusionPose[1]),2))+","+str(heading)+","+str(yawrate)+","+IMUStatus+","
    
    for c in PAOGI:
        csum ^= ord(c)
    #print hex(csum)
    csum=str(hex(csum))[-2:]
    csum= csum.upper()
    PAOGI = "$"+PAOGI+"*"+csum
    #print PAOGI
      ### NEED TO REMOVE THE 0x from teh start and go to all caps on the letters!!!
    return(PAOGI)
    
##Usage of the utm function
##import utm
##Convert a (latitude, longitude) tuple into an UTM coordinate:
##
##utm.from_latlon(51.2, 7.5)
##>>> (395201.3103811303, 5673135.241182375, 32, 'U')
##The syntax is utm.from_latlon(LATITUDE, LONGITUDE).
##
##The return has the form (EASTING, NORTHING, ZONE NUMBER, ZONE LETTER).
##
##Convert an UTM coordinate into a (latitude, longitude) tuple:
##
##utm.to_latlon(340000, 5710000, 32, 'U')
##>>> (51.51852098408468, 6.693872395145327)
##The syntax is utm.to_latlon(EASTING, NORTHING, ZONE NUMBER, ZONE LETTER).
##
##The return has the form (LATITUDE, LONGITUDE).
##
##Since the zone letter is not strictly needed for the conversion you may also the northern parameter instead, which is a named parameter and can be set to either True or False. Have a look at the unit tests to see how it can be used.
##
##The UTM coordinate system is explained on this Wikipedia page.
if __name__ == "__main__":
    try:
        
        while running:
            
            #print(imu.IMURead())   
            if imu.IMURead():
                #print("In IMU read")
                now=time.time()
                # x, y, z = imu.getFusionData()
                # print("%f %f %f" % (x,y,z))
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
                    #print "Roll: ", roll, "   KalRoll:", KalRoll, "   Delta:", roll-KalRoll
                    ## When this section is active it breaks the IMU read!!!        
            #print(read_GPS())
                GGA, RMC, GPS_Update=read_GPS()
                if GPS_Update:
                    current_GGA = GGA
                    current_RMC = RMC
                    #current_VTG = VTG
                    #print current_RMC
                    #print current_GGA
                    #compensatedLatLon = correct_for_angles(fusionPose[0], fusionPose[1], current_RMC, heading)
                    #send_TCPout(RMC, GGA, PAOGI)
                    #print(RMC.latitude, RMC.longitude)
                #time.sleep(max(poll_interval*.1/1000.0,0))

                
                #if(GPS_Update):
                #if ((now - last_update) > 1/update_rate):
                   #print("IMUstatus: ", IMUStatus)
                    
                    
                    #print(current_RMC.latitude, compensatedLatLon[0], current_RMC.longitude, compensatedLatLon[1])
                    last_update = now            
                    #print(now-last_update, " " , (now - last_update) > 1/update_rate, " ", 1/update_rate)
                    #RMC_out, GGA_out = output_NMEA(current_RMC, current_GGA, compensatedLatLon)
                    #print(RMC_out)
                    
                    #timestamp = current_RMC.split(",")[1]
                    
                    PAOGI = build_PAOGI(current_GGA, current_RMC, KalRoll, pitch, yaw, gyro, IMUStatus) 
                    #RMC_out_time = "%02d" % (current_RMC.timestamp.hour) + "%02d" % (current_RMC.timestamp.minute)+"%02d" % (current_RMC.timestamp.second)+"."+ "%02d" % ((current_RMC.timestamp.microsecond)/10000)
                    #output_string = PAOGI
                    #output_string = str(round(roll,2)) +","+str(round(pitch,2))+","+ str(round(heading,2))+","+ timestamp+","+str(KalRoll) #str(RMC_out.latitude)+","+ str(RMC_out.longitude)+","+str(RMC_out.spd_over_grnd)+","+str(RMC_out.true_course)+","+str(KalRoll)
                    #print "outtime: % 15.3f" %( time.time())
                    #print output_string
                    #message = "M_roll: ," + str(round(math.degrees(fusionPose[0]),2)) + " , M_pitch: ," + str(round(math.degrees(fusionPose[1]),2))+ " , M_heading: ," + str(round(heading,2))
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
                    #if((now-last_print) > 1*(1/update_rate)):
                #if GPS_Update:
                    #print("delta t: % 6.3f  r: % 5.2f p: % 5.2f  True_hdg % 5.2f  IMU Status  %s " % (now-last_print, math.degrees(fusionPose[0]), 
                    #    math.degrees(fusionPose[1]), heading, IMUStatus))#, use comma to not have a linefeed after this print
##                    #print"lat %12.9f lon: %12.9f" % (current_RMC.latitude, current_RMC.longitude),
##                    print"Corlat %12.9f Corlon: %12.9f" % (compensatedLatLon[0], compensatedLatLon[1]), now
                    
    ##                    if(GPS_Update != False):
    ##                        print(GPS_seconds, GGA.gps_qual, RMC.true_course, RMC.spd_over_grnd, RMC.latitude, RMC.longitude)
                        #print GPS_data
                        #print('delta t: {: 6.2f}'.format(now-last_print))
                    last_print = now
                    
                    #output_string = str(output_string)
                    #print output_string
                    if (save_to_file == True and started_file == True):
                        with open(file_name, 'a') as text_file:
                            text_file.write (output_string[:-4]+ str(roll)+"\r\n")

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
            time.sleep(0.01)
    ##          pass          
            
    except KeyboardInterrupt:
        print("interrupted!")
        GPS_sock.close()
        sock.close()
    
print (data)
    

#raw_input("Press key to exit\n") # Use input() in Python 3    
