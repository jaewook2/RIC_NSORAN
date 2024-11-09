 	 /*
==================================================================================
        Copyright (c) 2019-2020 AT&T Intellectual Property.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
==================================================================================
*/
/*
 * msgs_proc.hpp
 * Created on: 2019
 * Author: Ashwin Shridharan, Shraboni Jana
 */
#pragma once

#ifndef XAPP_MSG_XAPP_MSG_HPP_
#define XAPP_MSG_XAPP_MSG_HPP_

#include <iostream>
#include<rmr/rmr.h>
#include <rmr/RIC_message_types.h>
#include <mdclog/mdclog.h>

#include "a1_helper.hpp"
#include "e2ap_control.hpp"
#include "e2ap_control_response.hpp"
#include "e2ap_indication.hpp"
#include "subscription_delete_request.hpp"
#include "subscription_delete_response.hpp"
#include "subscription_helper.hpp"
#include "subscription_request.hpp"
#include "subscription_request.hpp"
#include "subscription_response.hpp"
#include "e2sm_subscription.hpp"
#include "subs_mgmt.hpp"
#include "../agent_connector.hpp"
// SEHONG - START
// - processing RIC indication header
#include <GlobalE2node-ID.h>
#include <GlobalE2node-eNB-ID.h>
#include <GlobalE2node-gNB-ID.h>
// - processing RIC indication message
#include <PM-Containers-Item.h>
#include <PF-Container.h>
#include <ODU-PF-Container.h>
#include <OCUCP-PF-Container.h>
#include <OCUUP-PF-Container.h>
#include <CellResourceReportListItem.h>
#include <PF-ContainerListItem.h>
#include <NRCGI.h>
#include <SlicePerPlmnPerCellListItem.h>
#include <FQIPERSlicesPerPlmnPerCellListItem.h>
#include <PerQCIReportListItem.h>
#include <ServedPlmnPerCellListItem.h>
#include <FGC-DU-PM-Container.h>
#include <EPC-DU-PM-Container.h>
#include <CUUPMeasurement-Container.h>
#include <FGC-CUUP-PM-Format.h>
#include <SliceToReportListItem.h>
#include <FQIPERSlicesPerPlmnListItem.h>
#include <EPC-CUUP-PM-Format.h>
#include <PerQCIReportListItemFormat.h>
#include <PlmnID-Item.h>
#include <PM-Info-Item.h>
#include <MeasurementType.h>
#include <MeasurementValue.h>
#include <L3-RRC-Measurements.h>
#include <RRCEvent.h>
#include <ServingCellMeasurements.h>
#include <MeasResultServMOList.h>
#include <MeasResultServMO.h>
#include <MeasResultNR.h>
#include <MeasQuantityResults.h>
#include <E2SM-KPM-IndicationMessage-Format1.h>
#include <E2SM-KPM-IndicationMessage.h>
#include <PerUE-PM-Item.h>
#include <ResultsPerSSB-IndexList.h>
#include <ResultsPerCSI-RS-IndexList.h>
#include <MeasResultNeighCells.h>
#include <MeasResultListNR.h>

#include <time.h>
#include <math.h>

#include "e2sm_indication.hpp"

#include <fstream> // jlee
#include <iostream> // jlee
// SEHONG - END

#define MAX_RMR_RECV_SIZE 2<<15

// SEHONG - START
#define REVERSE_SHORT(n) ((unsigned short) (((n & 0xFF) << 8) | \
                                            ((n & 0xFF00) >> 8)))
#define REVERSE_LONG(n) ((unsigned long) (((n & 0xFF) << 24) | \
                                          ((n & 0xFF00) << 8) | \
                                          ((n & 0xFF0000) >> 8) | \
                                          ((n & 0xFF000000) >> 24)))
#define REVERSE_64LONG(n) ((uint64_t)  (((n & 0xFF) << 56) | \
                                        ((n & 0xFF00) << 40) | \
                                        ((n & 0xFF0000) << 24) | \
                                        ((n & 0xFF000000) << 8) | \
                                        ((n & 0xFF00000000) >> 8) | \
                                        ((n & 0xFF0000000000) >> 24) | \
                                        ((n & 0xFF000000000000) >> 40) | \
                                        ((n & 0xFF00000000000000) >> 56)))

typedef struct {
	int64_t     TVsec;
	int64_t     TVnsec;
} Timestamp;

typedef struct {
    uint64_t        MetricType; // This value should be 0xF0F0---------
    uint64_t        MeasUnixTime_msec; 
    //Timestamp       MeasTimeStampPDCPBytes;
    int64_t         UsedPRBs; // 1-st info.
    //int64_t         PDCPBytesDL;
    //int64_t         PDCPBytesUL;
    //Timestamp       MeasTimestampPRB;
    //int64_t         AvailPRBDL;
    //int64_t         AvailPRBUL;
    //int64_t         MeasPeriodPDCP;
    //int64_t         MeasPeriodPRB;
    //char            nRPlmnID[];
    //char            nRCellID[];
    int64_t         TotDLAvailPRBs; // 2-nd info
    //int64_t         TotalofAvailablePRBsUL; 
    int64_t         TotNumServUEs;
    short           bitmap_servUEs; // 3-rd info
    char            CellID[8]; // 4-th info
} CellMetricsEntry;

typedef struct {
    int64_t     RSRP;
    int64_t     RSRQ;
    int64_t     RSSINR;
} CellRFType;

typedef struct {
    uint64_t          CellID;
    CellRFType      CellRF;
} NeighborCellRFType;

typedef struct {
    uint64_t            MetricType; // This value should be 0x1010-----
    uint64_t            MeasUnixTime_msec;
    int32_t             MatchedUEsTotNum;
    int32_t             RemainingUEsCount;
    uint64_t            UeID; // 1-st info.
    //Timestamp           MeasTimestampPDCPBytes;
    double              DL_throughput; // 2-nd info.
    //int64_t             PDCPBytesULUL;
    //Timestamp           MeasTimestampPRB;
    //int64_t             PRBUsageDL;
    //int64_t             PRBUsageUL;
    //Timestamp           MeasTimeRF;
    char            ServingCellID[8]; //3-rd info.
    CellRFType          ServingCellRF;    //4-th info.
    NeighborCellRFType  NeighborCell1_RF; //5-th info.
    NeighborCellRFType  NeighborCell2_RF; //6-th info.
    NeighborCellRFType  NeighborCell3_RF; //7-th info.

    //int64_t             QCI;
    //int64_t             FiveQI;
    //string              CellID;
    //int64_t             MeasPeriodRF; // in bronze version
    //int64_t             MeasPeriodPRB;
    //int64_t             MeasPeriodPDCP;
    //int64_t             NumberOfActiveUEs;
    //char                PDCPBytesDL[]; //3 bytes
    //char                PDCPBytesUL[]; //1 bytes
    //char                nRPlmnID[];
    //char                nRCellID[];
} UeMetricsEntry;

typedef struct {
    int64_t             MetricType;
    int64_t             UeID;
    int64_t             PDCPBytesDL;
    CellRFType          servingCellRF;
    char                NRCellID[8];
} testMetricsEntry;

// SEHONG - END

class XappMsgHandler{

private:
	std::string xapp_id;
	SubscriptionHandler *_ref_sub_handler;

    //std::map < std::string, bool> map_cuCpFilebool; 
    //std::map < std::string, bool> map_cuUpFilebool; 
    //std::map < std::string, bool> map_duFilebool;
    
    //std::string check_trace (std::string m_cellId, uint64_t  m_type);

public:
	//constructor for xapp_id.
	 XappMsgHandler(std::string xid){xapp_id=xid; _ref_sub_handler=NULL;};
	 XappMsgHandler(std::string xid, SubscriptionHandler &subhandler){xapp_id=xid; _ref_sub_handler=&subhandler;};

	 void operator() (rmr_mbuf_t *, bool*);

	 void register_handler();
	 bool encode_subscription_delete_request(unsigned char*, size_t* );

	 bool decode_subscription_response(unsigned char*, size_t );

	 bool a1_policy_handler(char *, int* , a1_policy_helper &);

	 void testfunction() {std::cout << "<<<<<<<<<<<<<<<<<<IN TEST FUNCTION<<<<<<<<<<<<<<<" << std::endl;}
    // log file을 위한 data 전송

};


void process_ric_indication(int message_type, transaction_identifier id, const void *message_payload, size_t message_len);
uint8_t procRicIndication(E2AP_PDU_t *e2apMsg, transaction_identifier gnb_id);
void DumpHex(char* data, int size);
//void ParseTimestamp(Timestamp *timestamp, char* timedata, int size);
void swap(char *A, char *B);
char *my_reverse(char *buffer, int i, int j);
char* itoa10(int value, char *buffer);
//char* ParsePLMNIdentity(uint8_t *data, int size, char *ret_Plmnid);
char* ParsePLMNIdentity(uint8_t *data, int size);
//char* ParseNRCGI(NRCGI_t nRCGI, char *ret_CellID);
char* ParseNRCGI(NRCGI_t nRCGI);



#endif /* XAPP_MSG_XAPP_MSG_HPP_ */
