import socket
import numpy as np
import pandas as pd
import torch
import os
import torch.nn as nn
import torch.optim as optim
import math
import shutil
from loging import LogMon
from localization import PossApp

host_ip = "10.0.2.1"
port = 4200
labelFileName = '/home/boo/nsOran/ETRITEST/1111/ues.txt'
model_name ='/home/boo/RIC_NSORAN/local_agent/AI_model/model_5sec_noscaler_converted.pth'
possPathName = "/home/boo/RIC_NSORAN/local_agent/posstrace"
possFileName = "Possition.txt"
log_path = '/home/boo/RIC_NSORAN/local_agent/logfiles'
monService = True
possService = True
window_size =  2*10**3

def msgs_proc(data_ue):
    # msg ==> "msg: cellId.imsi.msg_type,time_stamp,~" 형태로 들어옴
    # data_ue 는 cellId.imsi.msg_tytpe.time_stamp,~" 형태
    data_str= data_ue.split(sep='.') #[cellId, imsi, msg_tytpe,time_stamp,~]
    int_cellId = int(data_str[0])
    msg_cellId = str(int_cellId)
    msg_imsi = int(data_str[1]) 
    msg_type = data_str[2]
    msg_info = data_str[3]
    msg_time = float (msg_info.split(sep=',')[0])

    return msg_type, msg_time, msg_imsi, int_cellId, msg_info



def main():
    # defin for preprosseing  data 함수로 변경
    # monService
    # trace file 새로 만들기

    print('Runing Intelligent Localization Agent')
    
    print ("Enable Log and Monitoring Functions")
      
    # load AI model
    print ('Load AI model for infering possition')
    if monService == True:
        xAppLM = LogMon(log_path)
        xAppLM.init_logfiles()
    if possService == True:
        xAppPoss = PossApp( window_size=window_size, fileName=possFileName, trace_path =possPathName, model_name=model_name, labelFileName=labelFileName)
        xAppPoss.init_possfiles()
        

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
        print (input_data)
        for data_ue in input_data[1:]:
            # Message 분석 기능 
            msg_type, msg_time, msg_imsi, msg_cellId, msg_info = msgs_proc(data_ue)
            if monService ==True:
                xAppLM.writeTrace (msg_type, msg_cellId, msg_info)
            if possService == True:
                if msg_type == "cucp" and msg_cellId > 1:
                    xAppPoss.infernece_poss(msg_info,msg_imsi, msg_time)    


if __name__ == '__main__':
    main()

