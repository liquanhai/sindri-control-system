# Boost Converter
import csv, xlrd
from functools import reduce
## this class takes in messages from boost converter and
## translate them into human language
## input: excel file named "Code.xls" contains messages from boost converter
## output: None, print on screen

class BoostConverter:
    def __init__(self):
        wb = xlrd.open_workbook("CAN_DATA/data.xls")
        self.dct = {}
        dctName = ["MSGdict","statusDict","errorDict"]
        for i in range(1):
            self.dct[dctName[i]]={}
            sh = wb.sheet_by_index(i)
            for j in range(sh.nrows):
                if i==0:
                    try:
                        cell_value_code = str(int(sh.cell(j,0).value))
                    except:
                        cell_value_code = sh.cell(j,0).value
                    else:
                        cell_value_code = sh.cell(j,0).value
                        cell_value_name = sh.cell(j,1).value
                        self.dct[dctName[i]][cell_value_code] = cell_value_name
        self.readCSV("CAN_DATA/data.csv")


    def readCSV(self,filename):
        with open(filename) as csvfile:
            info = csv.DictReader(csvfile)
            count = 0
            for row in info:
                count += 1
                message = ""
                frameID = row["Frame ID"]
                data = row["Datas(Hex)"]
                mList = list(frameID)
                print(mList)
                dList = data.split()
                if mList[3]=='0':
                    message += "Direction: Send; "
                else:
                    message += "Direction: Recieve; "
                    message = message + self.dct["MSGdict"][mList[-1]] + ": "

                    if mList[-1] =="0":
                        print(count)

                    elif mList[-1] == "1":
                        message = message + str(int((dList[0] + dList[1]),16)) + "; "

                    elif mList[-1] =="2":
                        message = message + "Input Voltage:" + str(int((dList[0] + dList[1]),16)/100) + "V, "
                        message = message + "Input Current:" + str(int((dList[2] + dList[3]),16)/100) + "A, "
                        message = message + "Output Voltage:"+ str(int((dList[4] + dList[5]),16)/100) + "V, "
                        message = message + "Output Current:"+ str(int((dList[6] + dList[7]),16)/100) + "A. "

                    elif mList[-1] =="3":
                        message = message + "Internal Power Supply: " + str(int((dList[0] + dList[1]),16)/1000) + "V, "
                        message = message + "Coolant Temp:" + str(int((dList[2] + dList[3]),16)/100) + "C, "
                        message = message + "Internal Temp:"+ str(int((dList[4] + dList[5]),16)/100) + "C. "

                    elif mList[-1] =="B":
                        status = reduce((lambda x,y:x+y),dList[0:4])
                        error = reduce((lambda x,y:x+y),dList[4:])
                        try:
                            message = message+ "Status:"+self.dct["statusDict"][status]+ ";Error:"+self.dct["errorDict"][error]
                        except:
                            message = message + "Status:" + self.dct["statusDict"][status] + "; Error: " + "Unknown Error"
                    print(message)

a = BoostConverter()
