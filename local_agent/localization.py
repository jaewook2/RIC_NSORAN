import os
import shutil
import numpy as np
import pandas as pd
import torch
import os
import torch.nn as nn
import torch.optim as optim
import math

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

class PossApp ():
    def __init__(self, window_size =  2*10**3, fileName = "Possition.txt", trace_path ="", model_name ='/AI_model/model_5sec_converted.pth', labelFileName = '/home/boo/nsOran/ETRITEST/1111/ues.txt', sendInf = False):
        self.Indec_dict = {"timestamp" :0 , "ueImsiComplete": 1, "numActiveUes" : 2, "DRB.EstabSucc.5QI.UEID (numDrb)" : 3, "DRB.RelActNbr.5QI.UEID (0)": 4, \
               "L3 serving Id(m_cellId)": 5,"UE (imsi)" : 6, "L3 serving SINR" : 7, "L3 serving SINR 3gpp" :8, \
               "L3 neigh Id 1 (cellId)" : 9, "L3 neigh SINR 1":10,"L3 neigh SINR 3gpp 1 (convertedSinr)":11, \
               "L3 neigh Id 2 (cellId)" :12, "L3 neigh SINR 2":13,"L3 neigh SINR 3gpp 2 (convertedSinr)":14, \
               "L3 neigh Id 3 (cellId)" :15, "L3 neigh SINR 3":16,"L3 neigh SINR 3gpp 3 (convertedSinr)":17, \
               "L3 neigh Id 4 (cellId)" :18, "L3 neigh SINR 4":19,"L3 neigh SINR 3gpp 4 (convertedSinr)":20, \
               "L3 neigh Id 5 (cellId)" :21, "L3 neigh SINR 5":22,"L3 neigh SINR 3gpp 5 (convertedSinr)":23, \
               "L3 neigh Id 6 (cellId)" :24, "L3 neigh SINR 6":25,"L3 neigh SINR 3gpp 6 (convertedSinr)":26, \
               "L3 neigh Id 7 (cellId)" :27, "L3 neigh SINR 7":28,"L3 neigh SINR 3gpp 7 (convertedSinr)":29, \
               "L3 neigh Id 8 (cellId)" :30, "L3 neigh SINR 8":31,"L3 neigh SINR 3gpp 8 (convertedSinr)":32}
        self.parsing_key = ["timestamp", "ueImsiComplete", "L3 serving Id(m_cellId)", "L3 serving SINR", "L3 neigh Id 1 (cellId)", "L3 neigh SINR 1", \
                "L3 neigh Id 2 (cellId)", "L3 neigh SINR 2", "L3 neigh Id 3 (cellId)", "L3 neigh SINR 3"]
        self.index = ['timestamp', 'ueImsiComplete', 'SNR1', 'SNR2', 'SNR3', 'SNR4']
        self.SNR_data = pd.DataFrame(np.zeros((1,len(self.index))),columns=self.index) # total data
        self.window_size = window_size
        self.fileName = fileName
        self.trace_path = trace_path
        self.model_name =model_name
        self.labelFileName = labelFileName
        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            print("cuda")
        else:
            self.device = torch.device("cpu")
            print("cpu")
        self.pXappModel = self.load_model()
        self.index_infer = ['timestamp', 'imsi', 'cellID', 'x', 'y', 'label_x', 'label_y', 'Acc']
        self.inferResult = pd.DataFrame(columns=self.index_infer)
        
        if sendInf:
            influx_host = "localhost"
            influx_port = 8086
            influx_user = 'admin'
            influx_password = 'admin'
            db_name = 'influx'
            
            self.client = InfluxDBClient(
                host=influx_host,
                port=influx_port,
                username=influx_user,
                password=influx_password,
                database=db_name)

        #client.create_database(db_name)

            
        
    def load_model(self):
        input_dim = 4; output_dim = 2 ;hid_dim = 512; n_layer = 6; dropout = 0.2
        pXappModel = MLP(input_dim, output_dim, hid_dim, n_layer, dropout)
        pXappModel.load_state_dict(torch.load(self.model_name, map_location=self.device))
        return pXappModel


    def init_possfiles(self):
        if os.path.exists(self.trace_path):
            shutil.rmtree(self.trace_path)
        os.mkdir(self.trace_path)
        
        trace_file = open(self.trace_path+'/'+self.fileName, "w")
        trace_file.write ("timestemp, imsi, cellID, x, y, label_x, label_y, accuracy (1-MSE)")
        trace_file.close()
        
    def infernece_poss (self, data_str,targetImsi, current_time):
        print ("Parsing Message for possitioning:"+data_str)
        parsingData, scellID = self.pars_Realloc_SNR (data_str)
        print ("Parsed Message for possitioning:", parsingData)
        self.insertPandas(parsingData)
        print ("Update SNRs DB: \n", self.SNR_data)
        targetData=self.createTarget (targetImsi, current_time)
        print ("Create Input Data", targetData, " for ", targetImsi, " UE at ", current_time)
        input_values = torch.tensor(targetData, dtype=torch.float32, device = self.device)
        with torch.inference_mode():
            output_values = self.pXappModel(input_values)
        x = output_values.numpy()[0]
        y = output_values.numpy()[1]
        label_x, label_y = self.labelValues (targetImsi)
        RMSE = math.sqrt((label_x-x)**2+(label_y-y)**2)
        print ("At time "+str(current_time)+", "+str(targetImsi)+" UE's inferencing possition: (" + str(x)+","+str(y)+") with RMSE: "+str(RMSE) ) 
        print ("Real possition: (" + str(label_x)+","+str(label_y)+")" ) 

        trace_file = open(self.trace_path+'/'+self.fileName, "a")
        trace_file.write ("\n"+str(current_time) + "," + str(targetImsi) + "," + str(scellID) + "," \
        + str(x) + "," + str(y)+","+str(label_x) + "," + str(label_y) + "," + str(RMSE))
        trace_file.close()
        
        self.inferResult.loc[len(self.inferResult)] = [current_time, targetImsi, scellID, x, y, label_x, label_y, RMSE]
        print (self.inferResult)
    
    def pars_Realloc_SNR (self, data_str): 
        data_array = data_str.split(sep=',')
        output_array = np.zeros((6))
        for i in range(len(self.parsing_key)-1):
            key = self.parsing_key[i]
            if i < 2:
                output_array[i] = data_array[self.Indec_dict[key]]
            else:
                if i%2 == 0:
                    if i == 2: 
                        cellId = int(data_array[self.Indec_dict[key]])
                        scellID = cellId
                    else:
                        cellId = int(data_array[self.Indec_dict[key]])
                        
                    print ("Cell ID: ", cellId)
                    if cellId== 2:
                        snr_value_key = self.parsing_key[i+1]
                        output_array[2] = data_array[self.Indec_dict[snr_value_key]]
                    elif cellId == 3:
                        snr_value_key = self.parsing_key[i+1]
                        output_array[3] = data_array[self.Indec_dict[snr_value_key]]
                    elif cellId == 4:
                        snr_value_key = self.parsing_key[i+1]
                        output_array[4] = data_array[self.Indec_dict[snr_value_key]]
                    elif cellId == 5:
                        snr_value_key = self.parsing_key[i+1]
                        output_array[5] = data_array[self.Indec_dict[snr_value_key]]
                        
        return output_array, scellID


    def insertPandas(self, parsingData):
        pdData = pd.DataFrame(parsingData.reshape(1,len(self.index)), columns=self.index)
        self.SNR_data = pd.concat([self.SNR_data, pdData])
        
    def createTarget (self, targetImsi, current_time):
        ueSNR =  self.SNR_data.loc[( self.SNR_data.timestamp <= current_time) & ( self.SNR_data.timestamp >  0) &( self.SNR_data.ueImsiComplete == targetImsi),[ 'SNR1', 'SNR2', 'SNR3', 'SNR4']]
        avgSNR_value = np.mean(ueSNR.to_numpy(),  axis=0)            
        targetData = avgSNR_value
        print ("avgData = ", targetData)
        return targetData
    
    def labelValues (self, imsi):
        f = open(self.labelFileName)
        for line in f.readlines():
            values = line.split(sep=',')
            if int(values[0]) == imsi:
                x = values[1]
                y = values[2]
        f.close()
        return float(x), float(y) 
            
    def convertedSinr(self, sinr):
        sinr = float(sinr)
        inputEnd = float(40)
        inputStart = float(-23)   
        outputStart = float(0)
        outputEnd = float(127)
        slope = (outputEnd - outputStart) / (inputEnd - inputStart)
        
        if sinr == outputStart:
            outputSinr = inputStart
        elif sinr == outputEnd:
            outputSinr = inputEnd
        else:
            outputSinr = (sinr-outputStart) /  slope + inputStart
            
        return outputSinr
    '''
    def send_influx_db(self,):
        stat
        influx_point = {
                "measurement": stat,
                "tags": {
                    'timestamp':timestamp
                },
                "fields": {
                    "value": values[i]
                }
            }        
        self.client.write_points(influx_point)
    '''

