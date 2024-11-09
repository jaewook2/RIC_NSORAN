import socket
import numpy as np
import pandas as pd
import torch
import os
import torch.nn as nn
import torch.optim as optim
import math

host_ip = "10.0.2.1"
port = 4200
labelFileName = '/home/boo/nsOran/ETRITEST/1111/ues.txt'


cucp_index =  "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"\
             +"DRB.RelActNbr.5QI.UEID (0),L3 serving Id(m_cellId),UE (imsi),L3 serving SINR,"\
             +"L3 serving SINR 3gpp,"\
             +"L3 neigh Id 1 (cellId),L3 neigh SINR 1,L3 neigh SINR 3gpp 1 (convertedSinr),"\
             +"L3 neigh Id 2 (cellId),L3 neigh SINR 2,L3 neigh SINR 3gpp 2 (convertedSinr),"\
             +"L3 neigh Id 3 (cellId),L3 neigh SINR 3,L3 neigh SINR 3gpp 3 (convertedSinr),"\
             +"L3 neigh Id 4 (cellId),L3 neigh SINR 4,L3 neigh SINR 3gpp 4 (convertedSinr),"\
             +"L3 neigh Id 5 (cellId),L3 neigh SINR 5,L3 neigh SINR 3gpp 5 (convertedSinr),"\
             +"L3 neigh Id 6 (cellId),L3 neigh SINR 6,L3 neigh SINR 3gpp 6 (convertedSinr),"\
             +"L3 neigh Id 7 (cellId),L3 neigh SINR 7,L3 neigh SINR 3gpp 7 (convertedSinr),"\
             +"L3 neigh Id 8 (cellId),L3 neigh SINR 8,L3 neigh SINR 3gpp 8 (convertedSinr)"\

             
file_names =[]

if torch.cuda.is_available():
    device = torch.device("cuda")
    print("cuda")
else:
    device = torch.device("cpu")
    print("cpu")
             
def pars_Realloc_SNR (data_str, Indec_dict,parsing_key): 
    data_array = data_str.split(sep=',')
    output_array = np.zeros((6))
    for i in range(len(parsing_key)-1):
        key = parsing_key[i]
        if i < 2:
            output_array[i] = data_array[Indec_dict[key]]
        else:
            if i%2 == 0:
                if i == 2: 
                    cellId = int(data_array[Indec_dict[key]])- 1110
                    scellID = cellId
                else:
                    cellId = int(data_array[Indec_dict[key]])
                print ("Cell ID: ", cellId)
                if cellId== 2:
                    snr_value_key = parsing_key[i+1]
                    output_array[2] = data_array[Indec_dict[snr_value_key]]
                elif cellId == 3:
                    snr_value_key = parsing_key[i+1]
                    output_array[3] = data_array[Indec_dict[snr_value_key]]
                elif cellId == 4:
                    snr_value_key = parsing_key[i+1]
                    output_array[4] = data_array[Indec_dict[snr_value_key]]
                elif cellId == 5:
                    snr_value_key = parsing_key[i+1]
                    output_array[5] = data_array[Indec_dict[snr_value_key]]
    return output_array, scellID

def insertPandas(parsingData, SNR_data, index):
    pdData = pd.DataFrame(parsingData.reshape(1,len(index)), columns=index)
    SNR_data = pd.concat([SNR_data, pdData])
    return SNR_data

def createTarget (w_s, targetImsi, current_time, SNR_data):
    if current_time < w_s:
        ueSNR = SNR_data.loc[(SNR_data.timestamp <= current_time) &(SNR_data.ueImsiComplete == targetImsi),[ 'SNR1', 'SNR2', 'SNR3', 'SNR4']]
    else:
        startTime = 0 #current_time - w_s
        ueSNR = SNR_data.loc[(SNR_data.timestamp <= current_time) & (SNR_data.timestamp >  startTime) &(SNR_data.ueImsiComplete == targetImsi),[ 'SNR1', 'SNR2', 'SNR3', 'SNR4']]
    avgSNR_value = np.mean(ueSNR.to_numpy(),  axis=0)
    targetData = avgSNR_value
    print ("avgData = ", targetData)
    return targetData

def writeTrace (data_str, file_names):
    # parsing message format
    # meesage_type (0), 
    data_array = data_str.split(sep=',')
    message_type = data_array[0]
    fileName = ''
    int_cellId = int(data_array[6]) - 1110
    m_cellId = str(int_cellId)
    targetImsi = int(data_array[2]) 
    targetTime = float (data_array[1])
    if message_type == "du":
        fileName ="du-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('du,', '')
        indexs = cucp_index

    elif message_type == "cucp":
        fileName = "cu-cp-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('cucp,', '')
        indexs = cucp_index
    elif message_type == "cuup":
        fileName ="cu-up-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('cuup,', '')
        indexs = cucp_index

    if fileName not in file_names:
        print ((file_names))
        file_names.append(fileName)
        if os.path.exists(fileName):
            os.remove(fileName)
        trace_file = open(fileName, "w")
        trace_file.write (indexs)
        trace_file.close()
    # file check
    trace_file = open(fileName, "a")
    trace_file.write ('\n'+to_print)
    trace_file.close()
    return to_print, targetImsi, targetTime

def labelValues (labelFileName, imsi):
    f = open(labelFileName)
    for line in f.readlines():
        values = line.split(sep=',')
        if int(values[0]) == imsi:
            x = values[1]
            y = values[2]
    f.close()
    return float(x), float(y)


class MLP(nn.Module):
    def __init__(self, input_dim, output_dim, hid_dim, n_layer, dropout):
        super(MLP, self).__init__()
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.hid_dim = hid_dim
        self.n_layer = n_layer
        self.dropout = dropout

        self.fc1 = nn.Linear(self.input_dim, self.hid_dim)

        self.linears = nn.ModuleList()
        for i in range(self.n_layer-1):
            self.linears.append(nn.Linear(self.hid_dim, self.hid_dim))

        self.fc2 = nn.Linear(self.hid_dim, self.output_dim)
        self.act = nn.ReLU()
        self.dropout = nn.Dropout(self.dropout)

    def forward(self, x):
        x = self.act(self.fc1(x))
        
        for fc in self.linears:
            x = self.act(fc(x))
            x = self.dropout(x)
        
        x = self.fc2(x)

        return x

def main():
    # defin for preprosseing  data
    Indec_dict = {"timestamp" :0 , "ueImsiComplete": 1, "numActiveUes" : 2, "DRB.EstabSucc.5QI.UEID (numDrb)" : 3, "DRB.RelActNbr.5QI.UEID (0)": 4, \
               "L3 serving Id(m_cellId)": 5,"UE (imsi)" : 6, "L3 serving SINR" : 7, "L3 serving SINR 3gpp" :8, \
               "L3 neigh Id 1 (cellId)" : 9, "L3 neigh SINR 1":10,"L3 neigh SINR 3gpp 1 (convertedSinr)":11, \
               "L3 neigh Id 2 (cellId)" :12, "L3 neigh SINR 2":13,"L3 neigh SINR 3gpp 2 (convertedSinr)":14, \
               "L3 neigh Id 3 (cellId)" :15, "L3 neigh SINR 3":16,"L3 neigh SINR 3gpp 3 (convertedSinr)":17, \
               "L3 neigh Id 4 (cellId)" :18, "L3 neigh SINR 4":19,"L3 neigh SINR 3gpp 4 (convertedSinr)":20, \
               "L3 neigh Id 5 (cellId)" :21, "L3 neigh SINR 5":22,"L3 neigh SINR 3gpp 5 (convertedSinr)":23, \
               "L3 neigh Id 6 (cellId)" :24, "L3 neigh SINR 6":25,"L3 neigh SINR 3gpp 6 (convertedSinr)":26, \
               "L3 neigh Id 7 (cellId)" :27, "L3 neigh SINR 7":28,"L3 neigh SINR 3gpp 7 (convertedSinr)":29, \
               "L3 neigh Id 8 (cellId)" :30, "L3 neigh SINR 8":31,"L3 neigh SINR 3gpp 8 (convertedSinr)":32}

    parsing_key = ["timestamp", "ueImsiComplete", "L3 serving Id(m_cellId)", "L3 serving SINR", "L3 neigh Id 1 (cellId)", "L3 neigh SINR 1", \
                "L3 neigh Id 2 (cellId)", "L3 neigh SINR 2", "L3 neigh Id 3 (cellId)", "L3 neigh SINR 3", "L3 neigh Id 4 (cellId)", "L3 neigh SINR 4"]
    index = ['timestamp', 'ueImsiComplete', 'SNR1', 'SNR2', 'SNR3', 'SNR4']
    SNR_data = pd.DataFrame(np.zeros((1,len(index))),columns=index) # total data
    window_size =  2*10**3
    # trace file 새로 만들기
    fileName = "Possition.txt"
    if os.path.exists(fileName):
        os.remove(fileName)
    trace_file = open(fileName, "w")
    trace_file.write ("timestemp, imsi, cellID, x, y, label_x, label_y, accuracy (1-MSE)")
    trace_file.close()
    print('Runing Intelligent Localization Agent')
    
    # load AI model
    print ('Load AI model for infering possition')
    input_dim = 4; output_dim = 2 ;hid_dim = 512; n_layer = 6; dropout = 0.2
    pXappModel = MLP(input_dim, output_dim, hid_dim, n_layer, dropout)
    pXappModel.load_state_dict(torch.load('model_2sec_noscaler.pth', map_location=device))

    
    print ('Waiting for connection with xApp connector')
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host_ip, port))
    server.listen(5)

    control_sck, client_addr = server.accept()
    print('xApp connected: ' + client_addr[0] + ':' + str(client_addr[1]))
    # load AI model

    while True:
        data = control_sck.recv(4096)
        data_str = data.decode()
        print("Recived Data : ", data_str)
        print("logging")
        input_data = data_str.split(sep='msg:') # 1부터 시작
        for data_ue in input_data[1:]:
            to_print, targetImsi, current_time = writeTrace (data_ue, file_names) # for monitoring
            # ----- For positioning
            # preprosessing data
            parsingData, scellID  = pars_Realloc_SNR (to_print, Indec_dict,parsing_key)
            SNR_data = insertPandas(parsingData, SNR_data, index)
            targetData = createTarget (window_size, targetImsi, current_time, SNR_data)
            print("At time "+str(current_time)+", "+str(targetImsi)+" UE's SNRs:\n",SNR_data)
            print("At time "+str(current_time)+", "+str(targetImsi)+" UE's avg SNRs within time widow:\n",targetData) 
            
            # inferencing value
            input_values = torch.tensor(targetData, dtype=torch.float32, device = device)
            with torch.inference_mode():
                output_values = pXappModel(input_values)
            print (output_values) 
            x = output_values.numpy()[0]
            y = output_values.numpy()[1]
            
            # cal MSE
            label_x, label_y = labelValues (labelFileName, targetImsi)
            MSE = math.sqrt(np.mean((label_x-x)**2+(label_y-y)**2))

            print ("At time "+str(current_time)+", "+str(targetImsi)+" UE's inferencing possition: (" + str(x)+","+str(y)+") with MSE: "+str(MSE) ) 
            print ("Real possition: (" + str(label_x)+","+str(label_y)+")" ) 

            # graph 형태..            
            # create loging file () 
            trace_file = open(fileName, "a")
            trace_file.write ("\n"+str(current_time) + "," + str(targetImsi) + "," + str(scellID) + "," \
            + str(x) + "," + str(y)+","+str(label_x) + "," + str(label_y) + "," + str(MSE))
            trace_file.close()
        
        
if __name__ == '__main__':
    main()

