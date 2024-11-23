'''
cucp_5g_index =  "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"\
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

def writeTrace (data_str, log_path, file_names):
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
        if m_cellId == "1":
            indexs = cucp_4g_index

        else:
            indexs = cucp_5g_index
            
    elif message_type == "cuup":
        fileName ="cu-up-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('cuup,', '')
        if m_cellId == "1":
            indexs = cuup_4g_index

        else:
            indexs = cuup_5g_index
    
    if len(file_names) == 0: # First attempt to write log
        if os.path.exists(log_path):
            os.rmtree(log_path)
        os.mkdir(log_path)
            
    if fileName not in file_names:
        file_names.append(fileName)
        if os.path.exists(log_path+fileName):
            os.remove(log_path+fileName)
        trace_file = open(fileName, "w")
        trace_file.write (indexs)
        trace_file.close()
    # file check
    trace_file = open(log_path+fileName, "a")
    trace_file.write ('\n'+to_print)
    trace_file.close()
    
    return to_print, targetImsi, targetTime

'''

