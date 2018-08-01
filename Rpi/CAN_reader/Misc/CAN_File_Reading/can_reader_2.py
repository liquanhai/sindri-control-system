import pandas as pd
import numpy as np
from functools import reduce

df = pd.read_csv("../CAN_DATA/take2.csv")

# print(df)

df["Data"] = df["Datas(Hex)"].str.split(" ")

# print(df)

parsed_data = ["ID 1", "Input Voltage (V)", "Input Current (A)", \
"Output Voltage (V)", "Output Current (A)", "Internal Power Supply (V)", \
"Coolant Temp (C)", "Internal Temp (C)"]
n_df = pd.DataFrame(columns=parsed_data, index=range(len(df.Index)))

for index,row in df.iterrows():
    # print("Direction: ", row["Direction"])

    ID_type = row["Frame ID"][-1]
    data = row["Data"]
    if ID_type == "1":
        # print("ID 1: ", str(int((data[0] + data[1]), 16)))
        n_df.loc[index]["ID 1"] = str(int((data[0] + data[1]), 16))

    elif ID_type == "2":
        # print("Input Voltage: ", str(int((data[0] + data[1]), 16)/100), "V")
        # print("Input Current: " , str(int((data[2] + data[3]),16)/100), "A")
        # print("Output Voltage: ", str(int((data[4] + data[5]),16)/100), "V")
        # print("Output Current: ", str(int((data[6] + data[7]),16)/100), "A")
        n_df.loc[index]["Input Voltage (V)"]  = str(int((data[0] + data[1]), 16)/100)
        n_df.loc[index]["Input Current (A)"]  = str(int((data[2] + data[3]),16)/100)
        n_df.loc[index]["Output Voltage (V)"] = str(int((data[4] + data[5]),16)/100)
        n_df.loc[index]["Output Current (A)"] = str(int((data[6] + data[7]),16)/100)

    elif ID_type == "3":
        # print("Internal Power Supply: ",str(int((data[0] + data[1]),16)/1000), "V")
        # print("Coolant Temp: ", str(int((data[2] + data[3]),16)/100), "C")
        # print("Internal Temp: ", str(int((data[4] + data[5]),16)/100), "C")
        n_df.loc[index]["Internal Power Supply (V)"] = str(int((data[0] + data[1]),16)/1000)
        n_df.loc[index]["Coolant Temp (C)"] = str(int((data[2] + data[3]),16)/100)
        n_df.loc[index]["Internal Temp (C)"] = str(int((data[4] + data[5]),16)/100)

    elif ID_type == "B":
        status = reduce((lambda x,y:x+y),data[0:4])
        error = reduce((lambda x,y:x+y),data[4:])
        # try:
        #     "Status:"+self.dct["statusDict"][status]+ ";Error:"+self.dct["errorDict"][error]
        # except:
        #     message = message + "Status:" + self.dct["statusDict"][status] + "; Error: " + "Unknown Error"
        # print("(Status, Error)", status, error)
    # print(n_df)
    # n_df.loc[index] = parsed_d


n_df.to_csv("Parsed_data.csv", index=False, delimiter="\n")
