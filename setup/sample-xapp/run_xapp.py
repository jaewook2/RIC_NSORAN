import socket
import os

host_ip = "127.0.0.1"
AIPOSS_ip = "10.0.2.1"

port = 4200

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
             +"\n"

file_names =[]
def writeTrace (data_str):
    # parsing message format
    # meesage_type (0), 
    data_array = data_str.split(sep=',')
    message_type = data_str[0]
    m_cellId = data_str[6]
    if message_type == "1":
        fileName ="du-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('du,', '')
        indexs = cucp_index

    elif message_type == "2":
        fileName = "cu-cp-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('cucp,', '')
        indexs = cucp_index
    elif message_type == "3":
        fileName ="cu-up-cell-" + m_cellId + ".txt"
        to_print = data_str.replace('cuup,', '')
        indexs = cucp_index

    if fileName not in file_names:
        file_names.insert(fileName)
        if os.path.exists(fileName):
            os.remove(fileName)
        trace_file = open(fileName, "w")
        trace_file.write (indexs)
        trace_file.close()
    else:
        # file check
        trace_file = open(fileName, "a")
        trace_file.write (to_print)
        trace_file.close()
    return to_print

def main():
    # configure logger and console output
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # host = socket.gethostname()
    # bind to INADDR_ANY
    server.bind((host_ip, port))

    server.listen(5)

    control_sck, client_addr = server.accept()
    print('xApp connected: ' + client_addr[0] + ':' + str(client_addr[1]))
    
    while True:
        data = control_sck.recv(1024)
        print("Recived Data : ", data)
        writeTrace (data)

            
            
if __name__ == '__main__':
    main()

