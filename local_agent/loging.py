import os
import shutil

import logging

class LogMon ():
    def __init__(self, log_path):
        self.cucp4g_index = "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"\
                + "DRB.RelActNbr.5QI.UEID (0),enbdev (m_cellId),UE (imsi),sameCellSinr,"\
                + "sameCellSinr 3gpp encoded,L3 neigh Id (cellId),"\
                + "sinr,3gpp encoded sinr (convertedSinr)"
        self.cucp5g_index =  "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"\
             +"DRB.RelActNbr.5QI.UEID (0),L3 serving Id(m_cellId),UE (imsi),L3 serving SINR,"\
             +"L3 serving SINR 3gpp,"\
             +"L3 neigh Id 1 (cellId),L3 neigh SINR 1,L3 neigh SINR 3gpp 1 (convertedSinr),"\
             +"L3 neigh Id 2 (cellId),L3 neigh SINR 2,L3 neigh SINR 3gpp 2 (convertedSinr),"\
             +"L3 neigh Id 3 (cellId),L3 neigh SINR 3,L3 neigh SINR 3gpp 3 (convertedSinr),"\
             +"L3 neigh Id 4 (cellId),L3 neigh SINR 4,L3 neigh SINR 3gpp 4 (convertedSinr),"\
             +"L3 neigh Id 5 (cellId),L3 neigh SINR 5,L3 neigh SINR 3gpp 5 (convertedSinr),"\
             +"L3 neigh Id 6 (cellId),L3 neigh SINR 6,L3 neigh SINR 3gpp 6 (convertedSinr),"\
             +"L3 neigh Id 7 (cellId),L3 neigh SINR 7,L3 neigh SINR 3gpp 7 (convertedSinr),"\
             +"L3 neigh Id 8 (cellId),L3 neigh SINR 8,L3 neigh SINR 3gpp 8 (convertedSinr)"
        self.cuup4g_index = "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"\
                + "m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),"\
                + "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"\
                + "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),"\
                + "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"\
                + "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)"
        self.cuup5g_index = "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"\
                + "m_pDCPBytesUL (0)," "m_pDCPBytesDL (cellDlTxVolume),"\
                +"DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"\
                + "Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),"\
                + "DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"\
                + "(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)"
        self.file_names = []
        self.log_path = log_path
        


                
    def writeTrace (self, msg_type, msg_cellId, msg_info):
        # parsing message format
        # #cellId.imsi.msg_tytpe, 
        m_cellId = str(msg_cellId)
        fileName = ''
        print ("cell ID : "+ m_cellId)

        if msg_type == "du":
            fileName ="du-cell-" + m_cellId + ".txt"
            indexs = self.cucp5g_index

        elif msg_type == "cucp":
            fileName = "cu-cp-cell-" + m_cellId + ".txt"
            if msg_cellId > 1:
                indexs = self.cucp5g_index
            elif msg_cellId == 1:
                indexs = self.cucp4g_index
                
        elif msg_type == "cuup":
            fileName ="cu-up-cell-" + m_cellId + ".txt"
            if msg_cellId > 1:
                indexs = self.cuup5g_index
            elif msg_cellId == 1:
                indexs = self.cuup4g_index
                
        if fileName not in self.file_names:
            self.file_names.append(fileName)
            if os.path.exists(self.log_path+'/'+fileName):
                os.remove(self.log_path+'/'+fileName)
            trace_file = open(self.log_path+'/'+fileName, "w")
            trace_file.write (indexs)
            trace_file.close()
        # file check
        trace_file = open(self.log_path+'/'+fileName, "a")
        trace_file.write ('\n'+msg_info)
        trace_file.close()
             


    def init_logfiles(self):
        if os.path.exists(self.log_path):
            shutil.rmtree(self.log_path)
        os.mkdir(self.log_path)



