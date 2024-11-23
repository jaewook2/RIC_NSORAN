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
 * msgs_proc.cc
 * Created on: 2019
 * Author: Ashwin Shridharan, Shraboni Jana
 */


#include "msgs_proc.hpp"
#include <stdio.h>
// SEHONG
#include <stdlib.h>
#include <string.h>
//#include <unistd.h> // for sleep()



bool XappMsgHandler::encode_subscription_delete_request(unsigned char* buffer, size_t *buf_len){

	subscription_helper sub_helper;
	sub_helper.set_request(0); // requirement of subscription manager ... ?
	sub_helper.set_function_id(0);

	subscription_delete e2ap_sub_req_del;

	  // generate the delete request pdu

	  bool res = e2ap_sub_req_del.encode_e2ap_subscription(&buffer[0], buf_len, sub_helper);
	  if(! res){
	    mdclog_write(MDCLOG_ERR, "%s, %d: Error encoding subscription delete request pdu. Reason = %s", __FILE__, __LINE__, e2ap_sub_req_del.get_error().c_str());
	    return false;
	  }

	return true;

}

bool XappMsgHandler::decode_subscription_response(unsigned char* data_buf, size_t data_size){

	subscription_helper subhelper;
	subscription_response subresponse;
	bool res = true;
	E2AP_PDU_t *e2pdu = 0;

	asn_dec_rval_t rval;

	ASN_STRUCT_RESET(asn_DEF_E2AP_PDU, e2pdu);

	rval = asn_decode(0,ATS_ALIGNED_BASIC_PER, &asn_DEF_E2AP_PDU, (void**)&e2pdu, data_buf, data_size);
	switch(rval.code)
	{
		case RC_OK:
			   //Put in Subscription Response Object.
			   //asn_fprint(stdout, &asn_DEF_E2AP_PDU, e2pdu);
			   break;
		case RC_WMORE:
				mdclog_write(MDCLOG_ERR, "RC_WMORE");
				res = false;
				break;
		case RC_FAIL:
				mdclog_write(MDCLOG_ERR, "RC_FAIL");
				res = false;
				break;
		default:
				break;
	 }
	ASN_STRUCT_FREE(asn_DEF_E2AP_PDU, e2pdu);
	return res;

}

bool  XappMsgHandler::a1_policy_handler(char * message, int *message_len, a1_policy_helper &helper){

  rapidjson::Document doc;
  if (doc.Parse(message).HasParseError()){
    mdclog_write(MDCLOG_ERR, "Error: %s, %d :: Could not decode A1 JSON message %s\n", __FILE__, __LINE__, message);
    return false;
  }

  //Extract Operation
  rapidjson::Pointer temp1("/operation");
    rapidjson::Value * ref1 = temp1.Get(doc);
    if (ref1 == NULL){
      mdclog_write(MDCLOG_ERR, "Error : %s, %d:: Could not extract policy type id from %s\n", __FILE__, __LINE__, message);
      return false;
    }

   helper.operation = ref1->GetString();

  // Extract policy id type
  rapidjson::Pointer temp2("/policy_type_id");
  rapidjson::Value * ref2 = temp2.Get(doc);
  if (ref2 == NULL){
    mdclog_write(MDCLOG_ERR, "Error : %s, %d:: Could not extract policy type id from %s\n", __FILE__, __LINE__, message);
    return false;
  }
   //helper.policy_type_id = ref2->GetString();
    helper.policy_type_id = to_string(ref2->GetInt());

    // Extract policy instance id
    rapidjson::Pointer temp("/policy_instance_id");
    rapidjson::Value * ref = temp.Get(doc);
    if (ref == NULL){
      mdclog_write(MDCLOG_ERR, "Error : %s, %d:: Could not extract policy type id from %s\n", __FILE__, __LINE__, message);
      return false;
    }
    helper.policy_instance_id = ref->GetString();

    if (helper.policy_type_id == "1" && helper.operation == "CREATE"){
    	helper.status = "OK";
    	Document::AllocatorType& alloc = doc.GetAllocator();

    	Value handler_id;
    	handler_id.SetString(helper.handler_id.c_str(), helper.handler_id.length(), alloc);

    	Value status;
    	status.SetString(helper.status.c_str(), helper.status.length(), alloc);


    	doc.AddMember("handler_id", handler_id, alloc);
    	doc.AddMember("status",status, alloc);
    	doc.RemoveMember("operation");
    	StringBuffer buffer;
    	Writer<StringBuffer> writer(buffer);
    	doc.Accept(writer);
    	strncpy(message,buffer.GetString(), buffer.GetLength());
    	*message_len = buffer.GetLength();
    	return true;
    }
    return false;
 }


//For processing received messages.XappMsgHandler should mention if resend is required or not.
void XappMsgHandler::operator()(rmr_mbuf_t *message, bool *resend){

	if (message->len > MAX_RMR_RECV_SIZE){
		mdclog_write(MDCLOG_ERR, "Error : %s, %d, RMR message larger than %d. Ignoring ...", __FILE__, __LINE__, MAX_RMR_RECV_SIZE);
		return;
	}
	a1_policy_helper helper;
	bool res=false;
	switch(message->mtype){
		//need to fix the health check.
		case (RIC_HEALTH_CHECK_REQ):
				message->mtype = RIC_HEALTH_CHECK_RESP;        // if we're here we are running and all is ok
				message->sub_id = -1;
				strncpy( (char*)message->payload, "HELLOWORLD OK\n", rmr_payload_size( message) );
				*resend = true;
				break;

		case (RIC_INDICATION): {
			mdclog_write(MDCLOG_INFO, "Received RIC indication message of type = %d", message->mtype);
			unsigned char *me_id_null;
			unsigned char *me_id = rmr_get_meid(message, me_id_null);
			mdclog_write(MDCLOG_INFO,"RMR Received MEID: %s",me_id);
			process_ric_indication(message->mtype, me_id, message->payload, message->len);

			break;
		}

		case (RIC_SUB_RESP): {
        		mdclog_write(MDCLOG_INFO, "Received subscription message of type = %d", message->mtype);

				unsigned char *me_id_null;
				unsigned char *me_id = rmr_get_meid(message, me_id_null);
				mdclog_write(MDCLOG_INFO,"RMR Received MEID: %s",me_id);

				if(_ref_sub_handler !=NULL){
					_ref_sub_handler->manage_subscription_response(message->mtype, me_id, message->payload, message->len);
				} else {
					mdclog_write(MDCLOG_ERR, " Error :: %s, %d : Subscription handler not assigned in message processor !", __FILE__, __LINE__);
				}
				*resend = false;
				break;
		 }

	case A1_POLICY_REQ:

		    mdclog_write(MDCLOG_INFO, "In Message Handler: Received A1_POLICY_REQ.");
			helper.handler_id = xapp_id;

			res = a1_policy_handler((char*)message->payload, &message->len, helper);
			if(res){
				message->mtype = A1_POLICY_RESP;        // if we're here we are running and all is ok
				message->sub_id = -1;
				*resend = true;
			}
			break;

	default:
		{
			mdclog_write(MDCLOG_ERR, "Error :: Unknown message type %d received from RMR", message->mtype);
			*resend = false;
		}
	}

	return;

};

void process_ric_indication(int message_type, transaction_identifier id, const void *message_payload, size_t message_len) {

	std::cout << "In Process RIC indication" << std::endl;
	std::cout << "ID " << id << std::endl;

	// decode received message payload
	E2AP_PDU_t *pdu = nullptr;
	auto retval = asn_decode(nullptr, ATS_ALIGNED_BASIC_PER, &asn_DEF_E2AP_PDU, (void **) &pdu, message_payload, message_len);

	// print decoded payload
	if (retval.code == RC_OK) {
		char *printBuffer;
		size_t size;
		FILE *stream = open_memstream(&printBuffer, &size);
		asn_fprint(stream, &asn_DEF_E2AP_PDU, pdu);
		mdclog_write(MDCLOG_DEBUG, "Decoded E2AP PDU: %s", printBuffer);

		uint8_t res = procRicIndication(pdu, id);
	}
	else {
		std::cout << "process_ric_indication, retval.code " << retval.code << std::endl;
	}
}

/*
 * Handle RIC indication
 */
 // string create : log handler
 // std: 
 // std: 
uint8_t procRicIndication(E2AP_PDU_t *e2apMsg, transaction_identifier gnb_id)
{
   uint8_t idx;
   uint8_t ied;
   uint8_t ret = RC_OK;
   uint32_t recvBufLen;
   RICindication_t *ricIndication;
   RICaction_ToBeSetup_ItemIEs_t *actionItem;

   // SEHONG: Creation of Two Variables for UeMetricsEntry and CellMetricsEntry
   uint64_t gMeasUnixTime_msec = 0x0;
   char gCellID[8]="0000000";
   int gPFContainterType = 0;



   printf("\nE2AP : RIC Indication received ");
   ricIndication = &e2apMsg->choice.initiatingMessage->value.choice.RICindication;
   for (idx = 0; idx < ricIndication->protocolIEs.list.count; idx++)
   {
      switch(ricIndication->protocolIEs.list.array[idx]->id)
      {			
				case 29:  // RIC request ID
				{
					RICrequestID_t	 cur_RICrequestID = ricIndication->protocolIEs.list.array[idx]-> value.choice.RICrequestID;
					long ricrequestorid = cur_RICrequestID.ricRequestorID;
					long ricinstanceid = cur_RICrequestID.ricInstanceID;
					printf("ricRequestorID: %ld\n", ricrequestorid);
					printf("ricInstanceID: %ld\n", ricinstanceid);
					break;
				}
		        
				
				case 28:  // RIC indication type
				{
					long ricindicationType = ricIndication->protocolIEs.list.array[idx]-> value.choice.RICindicationType;
					printf("ricindicationType %ld\n", ricindicationType);
					break;
				}
				case 27:  // RIC indication SN
				{
					break;
				}
				case 26:  // RIC indication message
				{
					std::string agent_ip = find_agent_ip_from_gnb(gnb_id);  // dest. address for socket connection to drl_agent 

					struct tm *tm_info;
					time_t unix_timestamp;
					unix_timestamp = (time_t) (floor(gMeasUnixTime_msec/1000.0)); // msec --> sec
					tm_info = localtime(&unix_timestamp); //gmtime : localtime

					bool flag_ue = false; // this is indicator for writing data into influxDB, that is to send data on socket
					bool flag_cell = false; // this is indicator for writing data into influxDB, that is to send data on socket
					UeMetricsEntry *ueMetrics = NULL;
					CellMetricsEntry *cellMetrics = NULL;

					int payload_size = ricIndication->protocolIEs.list.array[idx]->value.choice.RICindicationMessage.size;
					char* payload = (char*) calloc(payload_size, sizeof(char));
					memcpy(payload, ricIndication->protocolIEs.list.array[idx]->value.choice.RICindicationMessage.buf, payload_size);

					E2SM_KPM_IndicationMessage_t* decodedMsg = e2sm_decode_ric_indication_message(payload, payload_size);

					unsigned long indMsg_present = (unsigned long) (decodedMsg->present);
					if (indMsg_present == 1) {
						E2SM_KPM_IndicationMessage_Format1_t *indMsgrFormat1 = decodedMsg->choice.indicationMessage_Format1;
						int count_PMContainer = int(indMsgrFormat1->pm_Containers.list.count);
						for (int i = 0; i < count_PMContainer; i++) { 							
							PM_Containers_Item_t pmContainer = *(PM_Containers_Item_t *)(indMsgrFormat1->pm_Containers.list.array[i]);
							if (pmContainer.performanceContainer != NULL) { // performanceContainer is optional
								int perfContainerType = (int)(pmContainer.performanceContainer->present);
								printf("message type: %d", perfContainerType);
								gPFContainterType = perfContainerType;
								if (perfContainerType == 1) {
									struct ODU_PF_Container	*oDU_PF = pmContainer.performanceContainer->choice.oDU;
									int count_oDUcellResourceReportListItem = oDU_PF->cellResourceReportList.list.count;
									// initial 
									// Cell Metric: PRBUsage (AvailablePRBs)
									if (flag_cell == false) {
										cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));
										flag_cell = true;
										cellMetrics->MetricType = gPFContainterType; 
										cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(cellMetrics->CellID, gCellID, strlen(gCellID));
									}
									//////////
									// Ue Metric: DlThp 
									if (flag_ue == false) {
										ueMetrics = (UeMetricsEntry *) malloc(sizeof(UeMetricsEntry));
										flag_ue = true;
										ueMetrics->MetricType = gPFContainterType; // Cell Metric for O-CUUP (gPFContainterType == 3)
										ueMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(ueMetrics->ServingCellID, gCellID, strlen(gCellID));
									}

									for (int j = 0; j < count_oDUcellResourceReportListItem; j++) {
										printf("%d-th oDU - cell Resource Report List Item processing \n", j+1);
										
										CellResourceReportListItem_t cellResourceReport = *(CellResourceReportListItem_t *)(oDU_PF->cellResourceReportList.list.array[j]);
										long plmnid_size = (long)(cellResourceReport.nRCGI.pLMN_Identity.size);
										char *test_plmnId = ParsePLMNIdentity(cellResourceReport.nRCGI.pLMN_Identity.buf, plmnid_size);
										long nrCellID_size = (long)(cellResourceReport.nRCGI.nRCellIdentity.size);
										int nrCellID_BitsUnsued_size = (int)(cellResourceReport.nRCGI.nRCellIdentity.bits_unused);
										char *test_CellID = ParseNRCGI(cellResourceReport.nRCGI);
										long stat_dl_TotalofAvailablePRBs =  -1;
										if (cellResourceReport.dl_TotalofAvailablePRBs != NULL) {
											stat_dl_TotalofAvailablePRBs =  (long)(*(cellResourceReport.dl_TotalofAvailablePRBs));
										} 
										// Cell Metric for O-DU
										if ((gPFContainterType == 1) && flag_cell) {
											cellMetrics->TotDLAvailPRBs = stat_dl_TotalofAvailablePRBs;
										}
										//////////

										long stat_ul_TotalofAvailablePRBs =  -1;
										if (cellResourceReport.ul_TotalofAvailablePRBs != NULL) {
											stat_ul_TotalofAvailablePRBs =  (long)(*(cellResourceReport.ul_TotalofAvailablePRBs));
										} 


										int count_ServedPlmnPerCellListItem = cellResourceReport.servedPlmnPerCellList.list.count;
										for (int k = 0; k < count_ServedPlmnPerCellListItem; k++) {
											ServedPlmnPerCellListItem_t servedPlmnPerCell = (ServedPlmnPerCellListItem_t)(*(cellResourceReport.servedPlmnPerCellList.list.array[k]));
											long served_plmnid_size = (long)(servedPlmnPerCell.pLMN_Identity.size);

											if (servedPlmnPerCell.du_PM_5GC != NULL) {
												int count_slicePerPlmnPerCellListItem = (int)(servedPlmnPerCell.du_PM_5GC->slicePerPlmnPerCellList.list.count);
												for (int l = 0; l < count_slicePerPlmnPerCellListItem; l++) {
													SlicePerPlmnPerCellListItem_t slicePerPlmnPerCell = *(SlicePerPlmnPerCellListItem_t *)(servedPlmnPerCell.du_PM_5GC->slicePerPlmnPerCellList.list.array[l]);
													int sliceID_sST_size = slicePerPlmnPerCell.sliceID.sST.size;

													if (slicePerPlmnPerCell.sliceID.sD != NULL) {
														int sliceID_sD_size = slicePerPlmnPerCell.sliceID.sD->size;
													}

													int count_FQIPERSlicesPerPlmnPerCell = int(slicePerPlmnPerCell.fQIPERSlicesPerPlmnPerCellList.list.count);
													for (int m = 0; m < count_FQIPERSlicesPerPlmnPerCell; m++) {
														FQIPERSlicesPerPlmnPerCellListItem_t fQIPerSlicePerPlmnPerCell = *(FQIPERSlicesPerPlmnPerCellListItem_t *)(slicePerPlmnPerCell.fQIPERSlicesPerPlmnPerCellList.list.array[m]);
														long val_fiveQI = fQIPerSlicePerPlmnPerCell.fiveQI;

														long val_FQI_dl_PRBUsage = -1;
														if (fQIPerSlicePerPlmnPerCell.dl_PRBUsage != NULL) {
															val_FQI_dl_PRBUsage = (long)(*(fQIPerSlicePerPlmnPerCell.dl_PRBUsage));
														} 
														//ueMetrics.PRBUsageDL = val_FQI_dl_PRBUsage;

														long val_FQI_ul_PRBUsage = -1;
														if (fQIPerSlicePerPlmnPerCell.ul_PRBUsage != NULL) {
															val_FQI_ul_PRBUsage = (long)(*(fQIPerSlicePerPlmnPerCell.ul_PRBUsage));
														} 
														//ueMetrics.PRBUsageUL = val_FQI_ul_PRBUsage;
													}
												}
											}

											if (servedPlmnPerCell.du_PM_EPC != NULL) {
												int count_perQCIReportListItem = (int)(servedPlmnPerCell.du_PM_EPC->perQCIReportList_du.list.count);
												for (int l = 0; l < count_perQCIReportListItem; l++) {
													PerQCIReportListItem_t perQCIReport = *(PerQCIReportListItem_t *)(servedPlmnPerCell.du_PM_EPC->perQCIReportList_du.list.array[l]);
													long val_qci = (long) (perQCIReport.qci);
													//ueMetrics.QCI = val_qci;

													long val_perQCIReport_dl_PRBUsage = -1;
													if (perQCIReport.dl_PRBUsage != NULL) {
														val_perQCIReport_dl_PRBUsage = (long)(*perQCIReport.dl_PRBUsage);
													} 

													long val_perQCIReport_ul_PRBUsage = -1;
													if (perQCIReport.ul_PRBUsage != NULL) {
														val_perQCIReport_ul_PRBUsage = (long)(*perQCIReport.ul_PRBUsage);
													} 
												}

											}
										}
									}
								} else if (perfContainerType == 2) {
									if (!flag_cell) {
										cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));
										flag_cell = true;
										cellMetrics->MetricType = gPFContainterType; // Cell Metric for O-CUUP (gPFContainterType == 3)
										cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(cellMetrics->CellID, gCellID, strlen(gCellID));
										cellMetrics->CellID[strlen(gCellID)] = '\0';
									}
									
									
									if (!flag_ue ) {
										ueMetrics = (UeMetricsEntry *) malloc(sizeof(UeMetricsEntry));
										flag_ue = true;
										ueMetrics->MetricType = gPFContainterType; // Ue Metric for O-CUCP (gPFContainterType == 2)
										ueMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(ueMetrics->ServingCellID, gCellID, strlen(gCellID));
										ueMetrics->ServingCellID[strlen(gCellID)] = '\0';
									}
									struct OCUCP_PF_Container	*oCUCP_PF = pmContainer.performanceContainer->choice.oCU_CP;
									long nActUEs = -1;
									if (oCUCP_PF->cu_CP_Resource_Status.numberOfActive_UEs != NULL) {
										nActUEs = *(long *)(oCUCP_PF->cu_CP_Resource_Status.numberOfActive_UEs);
									} 

								} else if (perfContainerType == 3) { //O-CUUP message
									if (!flag_cell) {
										cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));
										flag_cell = true;
										cellMetrics->MetricType = gPFContainterType; // Cell Metric for O-CUUP (gPFContainterType == 3)
										cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(cellMetrics->CellID, gCellID, strlen(gCellID));
										cellMetrics->CellID[strlen(gCellID)] = '\0';
									}

									if (!flag_ue ) {
										ueMetrics = (UeMetricsEntry *) malloc(sizeof(UeMetricsEntry));
										flag_ue = true;
										ueMetrics->MetricType = gPFContainterType; // Ue Metric for O-CUCP (gPFContainterType == 2)
										ueMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(ueMetrics->ServingCellID, gCellID, strlen(gCellID));
										ueMetrics->ServingCellID[strlen(gCellID)] = '\0';
									}
									
									struct OCUUP_PF_Container	*oCUUP_PF = pmContainer.performanceContainer->choice.oCU_UP;
									int count_oCUUPpfContainerListItem = oCUUP_PF->pf_ContainerList.list.count;
									for (int j = 0; j < count_oCUUPpfContainerListItem; j++) {
										PF_ContainerListItem_t oCUUP_PF_item =  *(PF_ContainerListItem_t *)(oCUUP_PF->pf_ContainerList.list.array[j]);
										long val_oCUUP_PF_interface_type = (long) (oCUUP_PF_item.interface_type);
										int count_oCUUP_PMContainer_plmnListItem= (int)(oCUUP_PF_item.o_CU_UP_PM_Container.plmnList.list.count);
										for (int k = 0; k < count_oCUUP_PMContainer_plmnListItem; k++) {
											PlmnID_Item_t cuUPPlmn = *(PlmnID_Item_t *)(oCUUP_PF_item.o_CU_UP_PM_Container.plmnList.list.array[k]);
											int cuUPPlmn_pLMN_ID_size = (int)(cuUPPlmn.pLMN_Identity.size);
											char *test_cuUPPlmn_pLMN_ID = ParsePLMNIdentity(cuUPPlmn.pLMN_Identity.buf, cuUPPlmn_pLMN_ID_size);

											if (cuUPPlmn.cu_UP_PM_5GC != NULL) {
												int count_sliceToReportListItem = (int)(cuUPPlmn.cu_UP_PM_5GC->sliceToReportList.list.count);
												for (int l = 0; l < count_sliceToReportListItem; l++) {
													SliceToReportListItem_t sliceToReportListItem = *(SliceToReportListItem_t *)(cuUPPlmn.cu_UP_PM_5GC->sliceToReportList.list.array[l]);
													int sliceID_sST_size2 = sliceToReportListItem.sliceID.sST.size;
													if (sliceToReportListItem.sliceID.sD != NULL) {
														int sliceID_sD_size2 = sliceToReportListItem.sliceID.sD->size;
													}
													int count_FQIPERSlicesPerPlmnList = int(sliceToReportListItem.fQIPERSlicesPerPlmnList.list.count);
													for (int m = 0; m < count_FQIPERSlicesPerPlmnList; m++) {
														FQIPERSlicesPerPlmnListItem_t fQIPerSlicePerPlmn = *(FQIPERSlicesPerPlmnListItem_t *)(sliceToReportListItem.fQIPERSlicesPerPlmnList.list.array[m]);	
														long val_fiveQI2 = fQIPerSlicePerPlmn.fiveQI;
													}
												}
											}

											if (cuUPPlmn.cu_UP_PM_EPC != NULL) {
												int count_perQCIReportList_cuup_Item = (int)(cuUPPlmn.cu_UP_PM_EPC->perQCIReportList_cuup.list.count);
												for (int l = 0; l < count_perQCIReportList_cuup_Item; l++) {
													PerQCIReportListItemFormat_t perQCIReport2 = *(PerQCIReportListItemFormat_t *)(cuUPPlmn.cu_UP_PM_EPC->perQCIReportList_cuup.list.array[l]);
													long val_drbqci = (long) (perQCIReport2.drbqci);
													if (perQCIReport2.pDCPBytesDL != NULL) {
														int size_perQCIReport_pDCPBytesDL = (int)(perQCIReport2.pDCPBytesDL->size);
														char chr_pDCPBytesDL[size_perQCIReport_pDCPBytesDL];
														strncpy((char *)chr_pDCPBytesDL, (char *)(perQCIReport2.pDCPBytesDL->buf), size_perQCIReport_pDCPBytesDL);
														if (size_perQCIReport_pDCPBytesDL == 2) { 
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 8) | ((int)chr_pDCPBytesDL[1]);
														} else if (size_perQCIReport_pDCPBytesDL == 3) { // in case of macro eNB
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 16) | (((int)chr_pDCPBytesDL[1]) << 8) | ((int)chr_pDCPBytesDL[2]);
														} else if (size_perQCIReport_pDCPBytesDL == 4) { // in case of macro eNB
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 24) | (((int)chr_pDCPBytesDL[1]) << 16) | (((int)chr_pDCPBytesDL[2]) << 8) | ((int)chr_pDCPBytesDL[3]);
														}
														//strncpy(ueMetrics.pDCPBytesDL, (char *)(perQCIReport2.pDCPBytesDL->buf), size_perQCIReport_pDCPBytesDL);
													} 
													if (perQCIReport2.pDCPBytesUL != NULL) {
														int size_perQCIReport_pDCPBytesUL = (int)(perQCIReport2.pDCPBytesUL->size);
													} 
												}
											}
										}
									}
								} else {
									printf("////// e2sm in unknown perfContainerType \n");
								}
							}
							// 1-2: Processing theRANContainer - NULL
							if (pmContainer.theRANContainer != NULL) { // theRANContainer is optional
								long theRANContainer_size = (long)(pmContainer.theRANContainer->size);
							}
						}

						
						// 3. Processing *list_of_PM_Information
						if (indMsgrFormat1->list_of_PM_Information != NULL) {
							int count_PMInfo = int(indMsgrFormat1->list_of_PM_Information->list.count);
							char tempMeasurementTypeName[30] = ""; // This is for RRU.PrbUsedDl of Cell Metric in O-DU
							long temp_PrbUsedDl = -1;

							for (int i = 0; i < count_PMInfo; i++) {
								PM_Info_Item_t pmInfo = *(PM_Info_Item_t *)(indMsgrFormat1->list_of_PM_Information->list.array[i]);
								// 3-1: pmType
								int pmInfo_type = (int)(pmInfo.pmType.present);
								if (pmInfo_type == 1) { // MeasurementType_PR_measName
									MeasurementTypeName_t val_measName = pmInfo.pmType.choice.measName;
									int val_measName_size = (int) (val_measName.size);
									strncpy(tempMeasurementTypeName, (char *)(val_measName.buf), val_measName_size);
									tempMeasurementTypeName[val_measName_size] = '\0';
									
								} else if (pmInfo_type == 2) { // MeasurementType_PR_measID
									long val_measID = (long)(pmInfo.pmType.choice.measID);
								} else {
									printf("Invalid pmInfo type \n");
								}

								// 3-2: pmVal
								int pmVal_type = (int)(pmInfo.pmVal.present);
								if (pmVal_type == 1) { // long	 valueInt
									long pmVal_valueInt = (long)(pmInfo.pmVal.choice.valueInt);
									temp_PrbUsedDl = pmVal_valueInt;
								} else if (pmVal_type == 2) { // double	 valueReal
									double pmVal_valueReal = (double)(pmInfo.pmVal.choice.valueReal);
								} else if (pmVal_type == 3) { // no value

								} else if (pmVal_type == 4) { // struct L3_RRC_Measurements	*valueRRC
									L3_RRC_Measurements_t rrcValue = *(L3_RRC_Measurements_t *)(pmInfo.pmVal.choice.valueRRC);
									long val_rrcEvent = (long)(rrcValue.rrcEvent);
									if (rrcValue.servingCellMeasurements != NULL) {
										int servCellMeasType = (int)(rrcValue.servingCellMeasurements->present);
										if (servCellMeasType == 1) { // struct MeasResultServMOList	*nr_measResultServingMOList;
											int count_nr_measResultServMO = (int)(rrcValue.servingCellMeasurements->choice.nr_measResultServingMOList->list.count);
											for (int j = 0; j < count_nr_measResultServMO; j++) {
												MeasResultServMO_t nr_measResultServingMO = *(MeasResultServMO_t *)(rrcValue.servingCellMeasurements->choice.nr_measResultServingMOList->list.array[j]);
												long measResServMO_servCellId = (long)(nr_measResultServingMO.servCellId);
												long val_physCellId = (long)(*(PhysCellId_t *)(nr_measResultServingMO.measResultServingCell.physCellId));

												if (nr_measResultServingMO.measResultServingCell.measResult.cellResults.resultsSSB_Cell != NULL) {
													struct MeasQuantityResults *val_resultsSSB_Cell = nr_measResultServingMO.measResultServingCell.measResult.cellResults.resultsSSB_Cell;
													long val_resSSBCell_rsrp = -1;
													if (val_resultsSSB_Cell->rsrp != NULL) {
														val_resSSBCell_rsrp = *(long *)(val_resultsSSB_Cell->rsrp);
													}

													long val_resSSBCell_rsrq = -1;
													if (val_resultsSSB_Cell->rsrq != NULL) {
														val_resSSBCell_rsrq = *(long *)(val_resultsSSB_Cell->rsrq);
													}

													long val_resSSBCell_sinr = -1;
													if (val_resultsSSB_Cell->sinr != NULL) {
														val_resSSBCell_sinr = *(long *)(val_resultsSSB_Cell->sinr);
													}
												}
											}
										}
									} else { printf("Invalid pmValue type \n"); }
								// Cell Metric: RRU.PrbUsedDl
									if (strncmp(tempMeasurementTypeName, "RRU.PrbUsedDl", strlen("RRU.PrbUsedDl")) == 0) { //MeasurementTypeName == "RRU.PrbUsedDl"
										if ((gPFContainterType == 1) && (flag_cell)) {
											cellMetrics->UsedPRBs = temp_PrbUsedDl;
										}
									}
								} // for loop END
							}
						}
						
						// 4. Processing *list_of_matched_UEs
						if (indMsgrFormat1->list_of_matched_UEs != NULL) {
							// Cell/Ue Metric: list of served UEs by serving Cell with Cell ID
							unsigned short indicatorMapUE = 0b0000000000000000;
							char strUEID[6] = "00000";
							int intUEID = 0;
							////////////////////////

							int count_matchedUEs = int(indMsgrFormat1->list_of_matched_UEs->list.count);
							//  Cell Metric 
							if (flag_cell) {
								cellMetrics->TotNumServUEs = count_matchedUEs;
							}
							//  Ue Metric 
							if (flag_ue) {
								ueMetrics->MatchedUEsTotNum = count_matchedUEs;
							}
							// UE Information
							for (int i = 0; i < count_matchedUEs; i++) {
								// Per UE information

								if (flag_ue) {
									ueMetrics->RemainingUEsCount = count_matchedUEs - (i + 1);
								}
								PerUE_PM_Item_t perUE_PM = *(PerUE_PM_Item_t *)(indMsgrFormat1->list_of_matched_UEs->list.array[i]);
								int size_ueID = (int)(perUE_PM.ueId.size);
								strncpy(strUEID, (char *)(perUE_PM.ueId.buf), size_ueID);
								intUEID = atoi(strUEID);
								if (flag_ue) {
									ueMetrics->UeID = intUEID;
								}
								///////////////////
								indicatorMapUE = indicatorMapUE | (1 << (intUEID-1));

								int count_PerUE_PM_info= int(perUE_PM.list_of_PM_Information->list.count);
								for (int j = 0; j < count_PerUE_PM_info; j++) {
									PM_Info_Item_t pmInfo_perUE_PM = *(PM_Info_Item_t *)(perUE_PM.list_of_PM_Information->list.array[j]);

									char ueMeasurementTypeName[30] = ""; // This is for DRB.UEThpDl.UEID of Ue Metric in O-DU
									double temp_DRB_UEThpDl_UEID = -1; // double

									// 4-1: pmType
									int pmInfo_type_perUE_PM = (int)(pmInfo_perUE_PM.pmType.present);
									if (pmInfo_type_perUE_PM == 1) { // MeasurementType_PR_measName
										MeasurementTypeName_t val_measName_perUE_PM = pmInfo_perUE_PM.pmType.choice.measName;
										int val_measName_size_perUE_PM = (int) (val_measName_perUE_PM.size);
										strncpy(ueMeasurementTypeName, (char *)(val_measName_perUE_PM.buf), val_measName_size_perUE_PM);
										ueMeasurementTypeName[val_measName_size_perUE_PM] = '\0';
										
									} else if (pmInfo_type_perUE_PM == 2) { // MeasurementType_PR_measID
										long val_measID_perUE_PM = (long)(pmInfo_perUE_PM.pmType.choice.measID);
									} else {
										printf("Invalid pmInfo type \n");
									}

									// 4-2: pmVal
									int pmVal_type_perUE_PM = (int)(pmInfo_perUE_PM.pmVal.present);
									if (pmVal_type_perUE_PM == 1) { // long	 valueInt
										long pmVal_valueInt_perUE_PM = (long)(pmInfo_perUE_PM.pmVal.choice.valueInt);
									} else if (pmVal_type_perUE_PM == 2) { // double	 valueReal
										double pmVal_valueReal_perUE_PM = (double)(pmInfo_perUE_PM.pmVal.choice.valueReal);
										temp_DRB_UEThpDl_UEID = pmVal_valueReal_perUE_PM; // this value may not be DRB_UEThpDl_UEID
									} else if (pmVal_type_perUE_PM == 3) { // no value

									} else if (pmVal_type_perUE_PM == 4) { // struct L3_RRC_Measurements	*valueRRC
										L3_RRC_Measurements_t rrcValue_perUE_PM = *(L3_RRC_Measurements_t *)(pmInfo_perUE_PM.pmVal.choice.valueRRC);
										long val_rrcEvent_perUE_PM = (long)(rrcValue_perUE_PM.rrcEvent);

										if (rrcValue_perUE_PM.servingCellMeasurements != NULL) {
											ServingCellMeasurements_t *SCM = (ServingCellMeasurements_t *)(rrcValue_perUE_PM.servingCellMeasurements);
											//int rrc_ServingCellRFellMeas = rrcValue_perUE_PM.servingCellMeasurements->present;
											int rrc_ServingCellRFellMeas = int(SCM->present);
											if (rrc_ServingCellRFellMeas == 1) { //nr_measResultServingMOList
												//int count_nr_measResultServMO1 = (int) (rrcValue_perUE_PM.servingCellMeasurements->choice.nr_measResultServingMOList->list.count);
												int count_nr_measResultServMO1 = int(SCM->choice.nr_measResultServingMOList->list.count);
												for (int k = 0; k < count_nr_measResultServMO1; k++) {
													//MeasResultServMO_t nr_measResultServingMO1 = *(MeasResultServMO_t *)(rrcValue_perUE_PM.servingCellMeasurements->choice.nr_measResultServingMOList->list.array[k]);
													MeasResultServMO_t nr_measResultServingMO1 = *(MeasResultServMO_t *)(SCM->choice.nr_measResultServingMOList->list.array[k]);
													long measResServMO_servCellId1 = (long)(nr_measResultServingMO1.servCellId);
													long val_physCellId1 = (long)(*(PhysCellId_t *)(nr_measResultServingMO1.measResultServingCell.physCellId));
													if (nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsSSB_Cell != NULL) {
														struct MeasQuantityResults *val_resultsSSB_Cell1 = nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsSSB_Cell;
														long val_resSSBCell_rsrp1 = -1;
														if (val_resultsSSB_Cell1->rsrp != NULL) {
															val_resSSBCell_rsrp1 = *(long *)(val_resultsSSB_Cell1->rsrp);
														}
														long val_resSSBCell_rsrq1 = -1;
														if (val_resultsSSB_Cell1->rsrq != NULL) {
															val_resSSBCell_rsrq1 = *(long *)(val_resultsSSB_Cell1->rsrq);
														}
														long val_resSSBCell_sinr1 = -1;
														if (val_resultsSSB_Cell1->sinr != NULL) {
															val_resSSBCell_sinr1 = *(long *)(val_resultsSSB_Cell1->sinr);
														}
														if ((gPFContainterType == 2) && (flag_ue)) {
															ueMetrics->ServingCellRF.RSSINR = val_resSSBCell_sinr1;
														}
													}
													if (nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsCSI_RS_Cell != NULL) {
													}
												}

											} else if (rrc_ServingCellRFellMeas == 2) { // struct MeasResultPCell	*eutra_measResultPCell;
												printf("TO-DO: rrc Serving Cell Measurements Type: eutra_measResultPCell \n");
											} else {
												printf("rrc unsupported Serving Cell Measurements Type \n");
											}
										}

										if (rrcValue_perUE_PM.measResultNeighCells != NULL) {
											MeasResultNeighCells_t *MRNC = (MeasResultNeighCells_t *)(rrcValue_perUE_PM.measResultNeighCells);
											int mRNCType = int(MRNC->present);
											if (mRNCType == 1) { //
												int count_measResultNR = int(MRNC->choice.measResultListNR->list.count);
												for (int k = 0; k < count_measResultNR; k++) {
													MeasResultNR_t *valMRNC = (MeasResultNR_t *)(MRNC->choice.measResultListNR->list.array[k]);
													uint64_t val_physCellId2 = (uint64_t)(*(PhysCellId_t *)(valMRNC->physCellId));

													if ((gPFContainterType == 2) && (flag_ue)) {
														if (k == 0) {
															ueMetrics->NeighborCell1_RF.CellID = val_physCellId2;
														} else if (k == 1) {
															ueMetrics->NeighborCell2_RF.CellID = val_physCellId2;
														} else if (k == 2) {
															ueMetrics->NeighborCell3_RF.CellID = val_physCellId2;
														} else {
															printf("TEST: value of %d-th Neigh. Cell ID (not supported)\n", k+1);
														}
													}
													///////////////////

													if (valMRNC->measResult.cellResults.resultsSSB_Cell != NULL) {
														struct MeasQuantityResults *val_resultsSSB_Cell2 = valMRNC->measResult.cellResults.resultsSSB_Cell;

														long val_resSSBCell_rsrp2 = -1;
														if (val_resultsSSB_Cell2->rsrp != NULL) {
															val_resSSBCell_rsrp2 = *(long *)(val_resultsSSB_Cell2->rsrp);
														}
														long val_resSSBCell_rsrq2= -1;
														if (val_resultsSSB_Cell2->rsrq != NULL) {
															val_resSSBCell_rsrq2 = *(long *)(val_resultsSSB_Cell2->rsrq);
														}
														long val_resSSBCell_sinr2 = -1;
														if (val_resultsSSB_Cell2->sinr != NULL) {
															val_resSSBCell_sinr2 = *(long *)(val_resultsSSB_Cell2->sinr);
														}
														if ((gPFContainterType == 2) && (flag_ue)) {
															if (k == 0) {
																ueMetrics->NeighborCell1_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
															} else if (k == 1) {
																ueMetrics->NeighborCell2_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
															} else if (k == 2) {
																ueMetrics->NeighborCell3_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
															} else {
																printf("TEST: value of %d-th Neigh. Cell RSSINR (not supported)\n", k+1);
															}
														}
														///////////////////
													}
													//printf("HERE 3 (k = %d)\n", k);
													if (valMRNC->measResult.cellResults.resultsCSI_RS_Cell != NULL) {
														printf("resultsCSI_RS_Cell1 \n");
													}
													//printf("HERE 4 (k = %d)\n", k);
												}

											} else if (mRNCType == 2) { //
												printf("TO-DO: mRNCType 2) \n");

											} else {
												printf("rrc unsupported measResultNeighCells Type \n");
											}
										}
									} else {
										printf("Invalid pmValue type for perUE_PM \n");
									}
									if (strncmp(ueMeasurementTypeName, "DRB.UEThpDl.UEID", strlen("DRB.UEThpDl.UEID")) == 0) { //MeasurementTypeName == "DRB.UEThpDl.UEID	"
										if ((gPFContainterType == 1) && (flag_ue)) {
											ueMetrics->DL_throughput = temp_DRB_UEThpDl_UEID;
										}
									}
								}
									/////////////////// 
								// Ue Metric: Sending the completed Ue Metric for a specific UE over socket
								if (flag_ue) {
									std::string m_cellId = ueMetrics->ServingCellID;
									int int_cellId = stoi (m_cellId);
									int_cellId = int_cellId - 1110;
									uint64_t imsi = ueMetrics->UeID;
									int64_t sinrThisCell = ueMetrics->ServingCellRF.RSSINR;										
									uint64_t neighcellId1 = ueMetrics->NeighborCell1_RF.CellID;
									int64_t neighcellSINR1 =ueMetrics->NeighborCell1_RF.CellRF.RSSINR;
									uint64_t neighcellId2 = ueMetrics->NeighborCell2_RF.CellID;
									int64_t neighcellSINR2 =ueMetrics->NeighborCell2_RF.CellRF.RSSINR;
									uint64_t neighcellId3 = ueMetrics->NeighborCell3_RF.CellID;
									int64_t neighcellSINR3 =ueMetrics->NeighborCell3_RF.CellRF.RSSINR;
									std::string reprot_type = "";
									std::string to_print = "msg:"+std::to_string(int_cellId)+"."+std::to_string(imsi)+".";
									if (ueMetrics->MetricType == 1){
										reprot_type = "du";
										printf ("Skip the processing DU message\n");
									} else if (ueMetrics->MetricType == 2) {
										printf("Parsing and Sending CU_CP message\n");
										reprot_type = "cucp";
										if (ueMetrics->ServingCellID == "1111"){
											/*
											 the string is timestamp, ueImsiComplete, numActiveUes, DRB.EstabSucc.5QI.UEID (numDrb), DRB.RelActNbr.5QI.UEID (0), enbdev (m_cellId),UE (imsi),sameCellSinr,"
                 							"sameCellSinr 3gpp encoded,L3 neigh Id (cellId)," "sinr,3gpp encoded sinr (convertedSinr)\n";
											*/
								      		to_print = to_print+reprot_type +"."+ std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," +\
											std::to_string (ueMetrics->MatchedUEsTotNum) + "," + "0" +","+"0"+  ","+ std::to_string(int_cellId) + ","  + std::to_string (imsi) + "," + std::to_string (sinrThisCell) + "," + "0";
										} else {
											/*
											*/
											to_print =to_print+ reprot_type +"."+ std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," +\
											std::to_string (ueMetrics->MatchedUEsTotNum) + "," + "0" +","+"0";

											std::string servingStr = ","+std::to_string(int_cellId)+ ","  + std::to_string (imsi) + "," + std::to_string (sinrThisCell) + "," + "0";
											std::string neighsinr = "," + std::to_string (neighcellId1) + "," + std::to_string (neighcellSINR1) + "," + "0"\
																+ "," + std::to_string (neighcellId2) + "," + std::to_string (neighcellSINR2) + "," + "0" \
																+"," + std::to_string (neighcellId3) + "," + std::to_string (neighcellSINR3) + "," + "0" \
																+"," + std::to_string (0) + "," + std::to_string (0) + "," + "0" \
																+"," + std::to_string (0) + "," + std::to_string (0) + "," + "0" \
																+"," + std::to_string (0) + "," + std::to_string (0) + "," + "0" \
																+"," + std::to_string (0) + "," + std::to_string (0) + "," + "0" \
																+"," + std::to_string (0) + "," + std::to_string (0) + "," + "0" ;
											to_print = to_print + servingStr +neighsinr;
											std::cout << to_print << std::endl;
										}
									} else if (ueMetrics->MetricType == 3) {
										printf("Parsing and Sending CU_UP message\n");

										reprot_type = "cuup";
										if (ueMetrics->ServingCellID == "1111"){
											/*
											// the string is timestamp,ueImsiComplete, DRB.PdcpSduDelayDl (cellAverageLatency),
											// m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
											// Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
											// DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
											// DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)

								      		std::string to_print = "msg:"+ reprot_type +","+ \
											std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," + "DRB.PdcpSduDelayDl (cellAverageLatency)" + "," + \
											"0" +","+"m_pDCPBytesDL"+  ","+ "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes)" + "," +\ 
											"Tot.PdcpSduNbrDl.UEID (txDlPackets)" + "," + "DRB.PdcpSduBitRateDl.UEID (pdcpThroughput)" + "," +\
											" DRB.PdcpSduDelayDl.UEID (pdcpLatency)" + "QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc)" + "," +\
											"DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)";
											*/
											to_print = to_print+ reprot_type +"."+ \
											std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," + "0" + "," + \
											"1" +","+"2"+  ","+ "3" + "," +\ 
											"4" + "," + "5" + "," +\
											"6" + "7" + "," +\
											"8";

										} else {
											/*
											// the string is timestamp,ueImsiComplete, DRB.PdcpSduDelayDl (cellAverageLatency),
											// m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),
											// Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),
											// DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc),
											// DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)
																			      		std::string to_print = "msg:"+ reprot_type +","+ \
											std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," + "DRB.PdcpSduDelayDl (cellAverageLatency)" + "," + \
											"0" +","+"m_pDCPBytesDL"+  ","+ "DRB.PdcpSduVolumeDl_Filter.UEID (txBytes)" + "," +\ 
											"Tot.PdcpSduNbrDl.UEID (txDlPackets)" + "," + "DRB.PdcpSduBitRateDl.UEID (pdcpThroughput)" + "," +\
											" DRB.PdcpSduDelayDl.UEID (pdcpLatency)" + "QosFlow.PdcpPduVolumeDL_Filter.UEID (txPdcpPduBytesNrRlc)" + "," +\
											"DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)";
											*/
											to_print = to_print+ reprot_type +"."+ \
											std::to_string ( ueMetrics->MeasUnixTime_msec) + "," + std::to_string (imsi) + "," + "0" + "," + \
											"8" +","+"7"+  ","+ "6" + "," +\ 
											"5" + "," + "4" + "," +\
											"3" + ","+ "2" + "," +\
											"1";
										}
									}

									if (ueMetrics->MetricType != 1) {
										mysend_socket_string(to_print, agent_ip);
									}
								}

								if (ueMetrics->RemainingUEsCount > 0) {
									//printf("TEST: another ue Metric shoud be delivered over socket\n");
								} else if (ueMetrics->RemainingUEsCount == 0) {
									flag_ue = false;
									free(ueMetrics);
								} 																		
							}
								//////////////////
							/// Cell Metric Sent!!!
							if ((flag_cell) && gPFContainterType == 3) {
								cellMetrics->bitmap_servUEs = indicatorMapUE;
								printf("TEST: indicatorMapUE: %04x\n", cellMetrics->bitmap_servUEs);
							}
						}						
					} else {
						printf("Unknonw RIC Indication Message Type \n");
					}
					e2sm_free_ric_indication_message(decodedMsg);
					free(payload);


					// print ue information
					if (flag_cell) {
						printf("Cell ID is %s (This value should not be <1111>)\n", gCellID);
						if (flag_cell) {
							// 
							printf("TEST: CellMetric Size to send: %d\n", sizeof(CellMetricsEntry));
							//mysend_socket((char *)cellMetrics, agent_ip, sizeof(CellMetricsEntry));
							printf("======== cellMetric sent over socket  =========\n");
							// DumpHex((char *)cellMetrics, sizeof(CellMetricsEntry));
							printf("CellMetric - MetricType: 0x%lX\n", cellMetrics->MetricType);
							printf("CellMetric - MeasUnixTime_msec: %lu\n", cellMetrics->MeasUnixTime_msec);
							printf("CellMetric - UsedPRBs: %ld\n", cellMetrics->UsedPRBs);
							printf("CellMetric - TotDLAvailPRBs: %ld\n", cellMetrics->TotDLAvailPRBs);
							printf("CellMetric - TotNumServUEs: %ld\n", cellMetrics->TotNumServUEs);
							printf("CellMetric - bitmap_servUEs: %x\n", cellMetrics->bitmap_servUEs);
							printf("CellMetric - CellID: %s\n", cellMetrics->CellID);
							uint64_t  m_type = cellMetrics->MetricType; // This value should be 0xF0F0---------
							uint64_t  m_time = cellMetrics->MeasUnixTime_msec; 
							int64_t   m_ues =cellMetrics->TotNumServUEs;
							std::string m_cellId = cellMetrics->CellID;

							std::string to_print = m_cellId + "," + std::to_string (m_type) + "," + \
							std::to_string (m_time) + "," + std::to_string (m_ues) +"\n";
							std::cout << to_print << std::endl;
							//char *to_send = NULL;
							//strcpy(to_send, to_print.c_str());
							//mysend_socket(to_send, agent_ip, to_print.length());
						}
						flag_cell = false;
						free(cellMetrics);
					}
					break;
				}
				case 25:  // RIC indication header for E2SM-KPM
				{

					int hdr_size = ricIndication->protocolIEs.list.array[idx]-> value.choice.RICindicationHeader.size;
					char* header = (char*) calloc(hdr_size, sizeof(char));
					memcpy(header, ricIndication->protocolIEs.list.array[idx]-> \
																		 value.choice.RICindicationHeader.buf, hdr_size);
					//printf("Header %s\n", header);
					printf("======== RIC indication header =========\n");

					E2SM_KPM_IndicationHeader_t* decodedHdr = e2sm_decode_ric_indication_header(header, hdr_size);
					unsigned long indHeader_present = (unsigned long) decodedHdr->present;
					if (indHeader_present == 1) {
						E2SM_KPM_IndicationHeader_Format1_t *indHeaderFormat1 = (decodedHdr->choice.indicationHeader_Format1);
						unsigned long colTimeStamp_size = (unsigned long)(indHeaderFormat1->collectionStartTime.size);
						uint64_t valTimestamp =*(uint64_t *)(indHeaderFormat1->collectionStartTime.buf);
						gMeasUnixTime_msec = REVERSE_64LONG(valTimestamp);	
						int KPMnodeIDType = (int) (indHeaderFormat1->id_GlobalE2node_ID.present);
						if (KPMnodeIDType == 1) {
							// To get Cell ID:
							char lCellID[8] = "0000000";
							printf("--- GlobalE2node_gNB_ID --- \n");
							printf("--- --- plmn_id --- --- \n");
							struct GlobalE2node_gNB_ID KPMnode = *(indHeaderFormat1->id_GlobalE2node_ID.choice.gNB);

							int gNB_plmn_id_size = (int) (KPMnode.global_gNB_ID.plmn_id.size);
							strncpy(lCellID, (char *)(KPMnode.global_gNB_ID.plmn_id.buf), gNB_plmn_id_size);
							
							int gNB_gnb_id_size = (int) (KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.size);
							int gNB_gnb_id_BitsUnused = (int) (KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.bits_unused);
							printf("--- --- gnb_id (Unused bits: %d) --- --- \n", gNB_gnb_id_BitsUnused);
							strncpy(lCellID+3, (char *)(KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.buf), 1);
							lCellID[4] = '\0';
							strncpy(gCellID, lCellID, strlen(lCellID));
							gCellID[4] = '\0';

						} else if (KPMnodeIDType == 2) {
							printf("GlobalE2node_en_gNB_ID \n");

						} else if (KPMnodeIDType == 3) {
							printf("GlobalE2node_ng_eNB_ID \n");

						} else if (KPMnodeIDType == 4) {
							// To get Cell ID:
							char lCellID[8] = "0000000";

							printf("GlobalE2node_eNB_ID \n");
							printf("--- --- plmn_id --- --- \n");
							struct GlobalE2node_eNB_ID KPMnode = *(indHeaderFormat1->id_GlobalE2node_ID.choice.eNB);
							int eNB_plmn_id_size = (int) (KPMnode.global_eNB_ID.pLMN_Identity.size);
							strncpy(lCellID, (char *)(KPMnode.global_eNB_ID.pLMN_Identity.buf), eNB_plmn_id_size);

							int eNB_Type = (int) (KPMnode.global_eNB_ID.eNB_ID.present);
							if (eNB_Type == 1) {
								int macro_eNB_id_size = (int) (KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.size);
								int macro_eNB_id_BitsUnused = (int) (KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.bits_unused);
								printf("--- --- macro_eNB_ID (Unused bits: %d) --- --- \n", macro_eNB_id_BitsUnused);
								DumpHex((char *)(KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.buf), macro_eNB_id_size);
								strncpy(lCellID+3, (char *)(KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.buf), 1);
								lCellID[4] = '\0';
								strncpy(gCellID, lCellID, strlen(lCellID));
								gCellID[4] = '\0';


							} else if (eNB_Type == 2) {
								printf("--- --- home_eNB_ID --- --- \n");
								
							} else if (eNB_Type == 3) {
								printf("--- --- short_Macro_eNB_ID --- --- \n");
								
							} else if (eNB_Type == 4) {
								printf("--- --- long_Macro_eNB_ID --- --- \n");
								
							}
							

						}
					} else {
						printf("Unknonw RIC Indication Header\n");
					}
					e2sm_free_ric_indication_header(decodedHdr);
					break;
				}
				case 20:  // RIC call processID
				{
					// TO DO
					break;
				}
				case 15:  //RIC acition ID
				{
					// TO DO
					break;
				}
				case 5:  // RAN Function ID for E2SM-KPM
				{
					// TO DO
					break;

				}
      }
   }
   return ret; // TODO update ret value in case of errors
}


/* SEHONG: HexDump for viewing packet format */
void DumpHex(char* data, int size) {
	char ascii[17];
	int i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		printf("%02X ", ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			printf(" ");
			if ((i+1) % 16 == 0) {
				printf("|  %s \n", ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					printf(" ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					printf("   ");
				}
				printf("|  %s \n", ascii);
			}
		}
	}
}

void swap(char *x, char *y) {
    char t = *x; *x = *y; *y = t;
}

char* my_reverse(char *buffer, int i, int j) {
    while (i < j) {
        swap(&buffer[i++], &buffer[j--]);
    }
 
    return buffer;
}

char *itoa10(int value, char* buffer) {
    int n = abs(value);
	//printf("1-TEST itoa10(), n: %d\n", n);
    int i = 0;
    while (n) {
        int r = n % 10;
        buffer[i++] = 48 + r;
        n = n / 10;
    }
	//printf("2-TEST itoa10(), n, i: %d %d\n", n, i);
    if (i == 0) {
        buffer[i++] = '0';
    }
 
    if (value < 0) {
        buffer[i++] = '-';
    }
    buffer[i] = '\0';
    //printf("3-TEST itoa10(), buffer: %s\n", buffer);
    return my_reverse(buffer, 0, i - 1);
}

char *ParsePLMNIdentity(uint8_t *data, int size) {
	char temp[6];
	char *PlmnID = (char *) calloc(7,sizeof (char));
	if (size != 3) {
		printf("////ERROR: e2sm entered ParsePLMNIdentity if size != 3 \n");
		return NULL;
	}

	uint8_t mcc[3];
	uint8_t mnc[3];

	mcc[0] = data[0] >> 4;
	mcc[1] = data[0] & 0xf;
	mcc[2] = data[1] >> 4;
	mnc[0] = data[1] & 0xf;
	mnc[1] = data[2] >> 4;
	mnc[2] = data[2] & 0xf;
	//printf("HERE1, mnc[0], data[1]: %d, %d\n", mnc[0], data[1]);
	//printf("TEST: int(mcc[0]): %d \n", int(mcc[0]));

	itoa10(int(mcc[0]), &temp[0]);
	itoa10(int(mcc[1]), &temp[1]);
	itoa10(int(mcc[2]), &temp[2]);
	itoa10(int(mnc[0]), &temp[3]);
	itoa10(int(mnc[1]), &temp[4]);
	itoa10(int(mnc[2]), &temp[5]);
	//printf("HERE2 \n");
	//printf("ParsePLMNIdentity() 1: mcc[0], int(mcc[0])%d, %d\n", mcc[0], int(mcc[0]));
	//printf("ParsePLMNIdentity() 1-1: int(temp[0]) %d\n", int(temp[0]));
	//printf("ParsePLMNIdentity() 1-1: temp[0] %c\n", temp[0]);

	if (mnc[0] == 0xf) {
		PlmnID[0] = temp[0];
		PlmnID[1] = temp[1];
		PlmnID[2] = temp[2];
		PlmnID[3] = temp[4];
		PlmnID[4] = temp[5];
		PlmnID[5] = '\0';
	} else {
		PlmnID[0] = temp[0];
		PlmnID[1] = temp[1];
		PlmnID[2] = temp[2];
		PlmnID[3] = temp[3];
		PlmnID[4] = temp[4];
		PlmnID[5] = temp[5];
		PlmnID[6] = '\0';
	}
	return PlmnID;
}

char *ParseNRCGI(NRCGI_t nRCGI) {
	char *CellID = (char *)calloc(16, sizeof(char));
	char temp[10] = {0};

	OCTET_STRING_t plmnID;
	BIT_STRING_t nrCellID;

	plmnID = nRCGI.pLMN_Identity;
	CellID = ParsePLMNIdentity(plmnID.buf, plmnID.size);
	int plmnID_size = strlen((char *)CellID);
	//printf("CellID in e2sm parsenrcgi func: %s, %d\n", (char *)CellID, plmnID_size);

	nrCellID = nRCGI.nRCellIdentity;
	//printf("plmnID.size = %ld \n", plmnID.size);
	//printf("nrCellID.size = %ld \n", nrCellID.size);

	if (plmnID.size != 3 || nrCellID.size != 5) {
		printf("Error: illegal length of NRCGI");
		return NULL;
	}

	uint8_t former[3];
	uint8_t latter[6];

	former[0] = nrCellID.buf[0] >> 4;
	former[1] = nrCellID.buf[0] & 0xf;
	former[2] = nrCellID.buf[1] >> 4;
	latter[0] = nrCellID.buf[1] & 0xf;
	latter[1] = nrCellID.buf[2] >> 4;
	latter[2] = nrCellID.buf[2] & 0xf;
	latter[3] = nrCellID.buf[3] >> 4;
	latter[4] = nrCellID.buf[3] & 0xf;
	latter[5] = nrCellID.buf[4] >> uint(nrCellID.bits_unused);

	itoa10(int(former[0]), &temp[0]);
	itoa10(int(former[1]), &temp[1]);
	itoa10(int(former[2]), &temp[2]);
	itoa10(int(latter[0]), &temp[3]);
	itoa10(int(latter[1]), &temp[4]);
	itoa10(int(latter[2]), &temp[5]);
	itoa10(int(latter[3]), &temp[6]);
	itoa10(int(latter[4]), &temp[7]);
	itoa10(int(latter[5]), &temp[8]);

	CellID[plmnID_size] = temp[0];
	CellID[plmnID_size+1] = temp[1];
	CellID[plmnID_size+2] = temp[2];
	CellID[plmnID_size+3] = temp[3];
	CellID[plmnID_size+4] = temp[4];
	CellID[plmnID_size+5] = temp[5];
	CellID[plmnID_size+6] = temp[6];
	CellID[plmnID_size+7] = temp[7];
	CellID[plmnID_size+8] = temp[8];
	CellID[plmnID_size+9] = '\0';

	return CellID;
}
/*
std::string check_trace (std::string m_cellId, uint64_t  m_type){
	if (m_type == 1) { // oDU
		bool bool_trace = map_duFilebool[m_cellId];
		std::string m_duFileName = "du-cell-" + m_cellId + ".txt"; 
		if (bool_trace ) {
			return m_duFileName;
		} else {
			std::ofstream csv {};
			csv.open (m_duFileName.c_str ());
			csv.close();
			map_duFilebool.insert(pair<std::string, bool>(m_cellId, true));
			return m_duFileName;
		}

	} else if (m_type == 2){ //OCUCP
		bool bool_trace = map_cuCpFilebool[m_cellId];
		std::string m_cuCpFileName = "cu-cp-cell-" + m_cellId + ".txt";

		if (bool_trace) {
			return m_cuCpFileName;
		} else {
			std::ofstream csv {};
			csv.open (m_cuCpFileName.c_str ());
			if (strcmp(m_cellId.c_str(), "1111") == 0 ) {
				csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
						"DRB.RelActNbr.5QI.UEID (0),enbdev (m_cellId),UE (imsi),sameCellSinr,"
						"sameCellSinr 3gpp encoded,L3 neigh Id (cellId),"
						"sinr,3gpp encoded sinr (convertedSinr)\n";
				csv.close();
			} else {
				csv << "timestamp,ueImsiComplete,numActiveUes,DRB.EstabSucc.5QI.UEID (numDrb),"
					"DRB.RelActNbr.5QI.UEID (0),L3 serving Id(m_cellId),UE (imsi),L3 serving SINR,"
					"L3 serving SINR 3gpp,"
					"L3 neigh Id 1 (cellId),L3 neigh SINR 1,L3 neigh SINR 3gpp 1 (convertedSinr),"
					"L3 neigh Id 2 (cellId),L3 neigh SINR 2,L3 neigh SINR 3gpp 2 (convertedSinr),"
					"L3 neigh Id 3 (cellId),L3 neigh SINR 3,L3 neigh SINR 3gpp 3 (convertedSinr),"
					"L3 neigh Id 4 (cellId),L3 neigh SINR 4,L3 neigh SINR 3gpp 4 (convertedSinr),"
					"L3 neigh Id 5 (cellId),L3 neigh SINR 5,L3 neigh SINR 3gpp 5 (convertedSinr),"
					"L3 neigh Id 6 (cellId),L3 neigh SINR 6,L3 neigh SINR 3gpp 6 (convertedSinr),"
					"L3 neigh Id 7 (cellId),L3 neigh SINR 7,L3 neigh SINR 3gpp 7 (convertedSinr),"
					"L3 neigh Id 8 (cellId),L3 neigh SINR 8,L3 neigh SINR 3gpp 8 (convertedSinr)"
					"\n";
				csv.close(); std::string m_cuCpFileName = "cu-cp-cell-" + m_cellId + ".txt";
			}
			map_cuCpFilebool.insert(pair<std::string, bool>(m_cellId, true));
			return m_cuCpFileName;
		}
	} else if (m_type == 3) { //OCUUP
		bool bool_trace = map_cuUpFilebool[m_cellId];
		std::string m_cuUpFileName = "cu-up-cell-" + m_cellId + ".txt";

		if (bool_trace) {
			return m_cuUpFileName;
		} else {
			std::ofstream csv {};
			csv.open (m_cuUpFileName.c_str ());
			if (strcmp(m_cellId.c_str(), "1111") == 0 ) {
				csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
						"m_pDCPBytesUL (0),m_pDCPBytesDL (cellDlTxVolume),"
						"DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
						"Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID (pdcpThroughput),"
						"DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
						"(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
				csv.close();
			} else {
				csv << "timestamp,ueImsiComplete,DRB.PdcpSduDelayDl (cellAverageLatency),"
					"m_pDCPBytesUL (0),"
					"m_pDCPBytesDL (cellDlTxVolume),DRB.PdcpSduVolumeDl_Filter.UEID (txBytes),"
					"Tot.PdcpSduNbrDl.UEID (txDlPackets),DRB.PdcpSduBitRateDl.UEID"
					"(pdcpThroughput),"
					"DRB.PdcpSduDelayDl.UEID (pdcpLatency),QosFlow.PdcpPduVolumeDL_Filter.UEID"
					"(txPdcpPduBytesNrRlc),DRB.PdcpPduNbrDl.Qos.UEID (txPdcpPduNrRlc)\n";
				csv.close ();
			}
			map_cuUpFilebool.insert(pair<std::string, bool>(m_cellId, true));
			return  m_cuUpFileName;
		}


	} else{
		printf("Unsupported Message Type\n");
	}
}
*/

