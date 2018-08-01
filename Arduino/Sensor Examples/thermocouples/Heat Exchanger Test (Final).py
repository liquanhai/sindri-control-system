import serial
import matplotlib.pyplot as plt
import csv
import time
print repr(time)
import getch




###### Helper Functions ######
def splitLine(line):
    listLine = line.split(',')
    return listLine



###### Core Functions ######

def openSerial():
    ## Open Serial Port
    print("Searching for Serial Ports")
    locations=['/dev/ttyACM0', '/dev/ttyACM1','/dev/ttyACM2', '/dev/ttyACM3','/dev/ttyACM4', '/dev/ttyACM5','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3', '/dev/ttyUSB4', '/dev/ttyUSB5', '/dev/ttyUSB6', '/dev/ttyUSB7', '/dev/ttyUSB8', '/dev/ttyUSB9', '/dev/ttyUSB10','/dev/ttyS0', '/dev/ttyS1', '/dev/ttyS2', 'com2', 'com3', 'com4', 'com5', 'com6', 'com7', 'com8', 'com9', 'com10', 'com11', 'com12', 'com13', 'com14', 'com15', 'com16', 'com17', 'com18', 'com19', 'com20', 'com21', 'com1', 'end']
    detectedLocations = []
    for device in locations:
        try:
            # timeout is the amount of time the serial will block the code to wait for the data, if it isn't received in 2 seconds, it will perceive as null data
            serialArduinoTest = serial.Serial(device, 115200, timeout=2)
            print(device + " is detected")
            detectedLocations.append(device)
            serialArduinoTest.close()
        except:
            if device == 'end' and len(detectedLocations) != 1:
                print ("Unable to find Serial Ports or more than 1 Serial Ports detected. Please check cable connections.")
                time.sleep(1)
                exit()
                
    print("Binding Serial Ports")
    for device in detectedLocations:
        global serialArduino
        serialArduino = serial.Serial(device, 115200, timeout=2)
        print(device + " is binded to Probe 1")

    print("Starting Serial Connection")

def userInput():
    print("\nInput FileName")
    name = input("---->  ")
    return name

def dataAcquisition(outFile):
    while(True):
        print("Press SPACE to start recording")
        key = getch.getch()
        if key == b" ":
            print("Initiating Data Acquisition")
            break

    ## Open file
    print("Opening ", outFile)
    file = open(outFile + '.csv', 'w')

    ## start timer
    start = time.clock()

    ## parameters for graphing
    fig1 = plt.figure()
    plt.xlabel('Time')
    plt.ylabel('Temperature')
    
    timeV = [0,0]
    tc1 = [0,0]
    tc2 = [0,0]
    tc3 = [0,0]
    tc4 = [0,0]
    tc5 = [0,0]
    tc6 = [0,0]
    tc7 = [0,0]
    tc8 = [0,0]

    ## plot legend
    plt.plot(timeV,tc1,'b', label = "tc1")
    plt.plot(timeV,tc2, 'g', label = "tc2")
    plt.plot(timeV,tc3, 'r', label = "tc3")
    plt.plot(timeV,tc4, 'c', label = "tc4")
    plt.plot(timeV,tc5,'m', label = "tc5")
    plt.plot(timeV,tc6, 'y', label = "tc6")
    plt.plot(timeV,tc7, 'k', label = "tc7")
    plt.plot(timeV,tc8, 'teal', label = "tc8")
    plt.legend(bbox_to_anchor=(1.005, 1), loc=2, borderaxespad=0.)

    ## plot text boxes
    ymin, ymax = plt.gca().get_ylim()
    y_location = (ymax+ymin)/2
    x = (ymax-ymin)/20
    text1 = plt.text(timeV[1]+0.2,y_location, 'tc1: ' + str(tc1[1]))
    text2 = plt.text(timeV[1]+0.2,y_location-1*x, 'tc2: ' + str(tc2[1]))
    text3 = plt.text(timeV[1]+0.2,y_location-2*x, 'tc3: ' + str(tc3[1]))
    text4 = plt.text(timeV[1]+0.2,y_location-3*x, 'tc4: ' + str(tc4[1]))
    text5 = plt.text(timeV[1]+0.2,y_location-4*x, 'tc5: ' + str(tc5[1]))
    text6 = plt.text(timeV[1]+0.2,y_location-5*x, 'tc6: ' + str(tc6[1]))
    text7 = plt.text(timeV[1]+0.2,y_location-6*x, 'tc7: ' + str(tc7[1]))
    text8 = plt.text(timeV[1]+0.2,y_location-7*x, 'tc8: ' + str(tc8[1]))
    
    

    print("Flushing Buffer")
    serialArduino.flushInput() # flush the serial buffer
    now = 0
    while(True):

        ## check for ending of recording
        if(getch.getch() != None):
            key = getch.getch()
            if key == b" ":
                break

        ## data collection and storage
        print "Arduino Data..." 

        value = serialArduino.readline(500).decode('utf-8')[:-1]
        
        print str(value)

        ##previous = time.perf_counter() - now
        now = time.clock()
        relativeTime = int(now - start)
        if value:  # If it isn't a blank line
            line = str(relativeTime) + str(value)
            ## print(line)
            file.write(line)

            ## data plot update UI
            splitValues = splitLine(value[1:-1])
            ## print(splitValues)
            timeV.append(relativeTime)
            tc1.append(int(splitValues[0]))
            tc2.append(int(splitValues[1]))
            tc3.append(int(splitValues[2]))
            tc4.append(int(splitValues[3]))
            tc5.append(int(splitValues[4]))
            tc6.append(int(splitValues[5]))
            tc7.append(int(splitValues[6]))
            tc8.append(int(splitValues[7]))

            timeV.pop(0)
            tc1.pop(0)
            tc2.pop(0)
            tc3.pop(0)
            tc4.pop(0)
            tc5.pop(0)
            tc6.pop(0)
            tc7.pop(0)
            tc8.pop(0)
            

            ##after1 = time.perf_counter()

            ##print("Plotting Graph")
            plt.plot(timeV,tc1,'b', label = "tc1")
            plt.plot(timeV,tc2, 'g', label = "tc2")
            plt.plot(timeV,tc3, 'r', label = "tc3")
            plt.plot(timeV,tc4, 'c', label = "tc4")
            plt.plot(timeV,tc5,'m', label = "tc5")
            plt.plot(timeV,tc6, 'y', label = "tc6")
            plt.plot(timeV,tc7, 'k', label = "tc7")
            plt.plot(timeV,tc8, 'teal', label = "tc8")

            ## update text and reset x limits
            text1.remove()
            text2.remove()
            text3.remove()
            text4.remove()
            text5.remove()
            text6.remove()
            text7.remove()
            text8.remove()
            
            ymin, ymax = plt.gca().get_ylim()
            y_location = (ymax+ymin)/2
            x = (ymax-ymin)/20
            text1 = plt.text(timeV[1]+0.2,y_location, 'tc1: ' + str(tc1[1]))
            text2 = plt.text(timeV[1]+0.2,y_location-1*x, 'tc2: ' + str(tc2[1]))
            text3 = plt.text(timeV[1]+0.2,y_location-2*x, 'tc3: ' + str(tc3[1]))
            text4 = plt.text(timeV[1]+0.2,y_location-3*x, 'tc4: ' + str(tc4[1]))
            text5 = plt.text(timeV[1]+0.2,y_location-4*x, 'tc5: ' + str(tc5[1]))
            text6 = plt.text(timeV[1]+0.2,y_location-5*x, 'tc6: ' + str(tc6[1]))
            text7 = plt.text(timeV[1]+0.2,y_location-6*x, 'tc7: ' + str(tc7[1]))
            text8 = plt.text(timeV[1]+0.2,y_location-7*x, 'tc8: ' + str(tc8[1]))

            plt.xlim(timeV[1]-10,timeV[1])
            
            
            plt.draw()
            plt.pause(0.01)
            ##print("Graph Plotted")
            ##after2 = time.perf_counter()

            ##print(previous,after1-now, after2-now)

            
            
        
    plt.close('all')
    file.close()
    
    
    

    

def main():

    ## open serial
    openSerial()

    ## User input?
    fileName = userInput()

    ## Data Acquisition
    dataAcquisition(fileName)

    

    




if __name__ == '__main__':
    try:
        main()
    except Exception as ex:
        print(ex)
        key = getch()
