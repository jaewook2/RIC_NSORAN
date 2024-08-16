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
 * TODO doxy
 * SEHONG: 
 *       1. (completed) processing additional protocol IEs of case 29 (RIC request ID), case 25 (RIC indication header), case 26 (RIC indication message) 
 *       2. (ongoing) building CellMetric and UE Metric 
 */
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
   //printf("gCellID initalization: %s\n", gCellID);
   //////////

   printf("\n(SEHONG___)E2AP : RIC Indication received ");
   ricIndication = &e2apMsg->choice.initiatingMessage->value.choice.RICindication;

   printf("protocolIEs elements %d\n", ricIndication->protocolIEs.list.count);

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
					long ricindicationType = ricIndication->protocolIEs.list.array[idx]-> \
																		 value.choice.RICindicationType;

					printf("ricindicationType %ld\n", ricindicationType);

					break;
				}
				case 27:  // RIC indication SN
				{
					// TO DO
					break;
				}
				case 26:  // RIC indication message
				{
					std::string agent_ip = find_agent_ip_from_gnb(gnb_id);  // dest. address for socket connection to drl_agent 

					struct tm *tm_info;
					time_t unix_timestamp;

					printf("MeasUnixTime (msec) in RIC Indication Header: \n");
					printf("unix time (ms): %lu, unix time (sec): %lu\n", gMeasUnixTime_msec, (uint64_t)(floor(gMeasUnixTime_msec/1000.0)));
						unix_timestamp = (time_t) (floor(gMeasUnixTime_msec/1000.0)); // msec --> sec
						tm_info = localtime(&unix_timestamp); //gmtime : localtime
						printf("Year: %d, Month: %d, Day: %d, Hour: %d, Minute: %d, Second: %d, msec: %lu\n",  tm_info->tm_year + 1900, \
						 																			tm_info->tm_mon + 1, \
																									tm_info->tm_mday, \
																									tm_info->tm_hour + 9,\
																									tm_info->tm_min,\
																									tm_info->tm_sec, \
																									(gMeasUnixTime_msec - (uint64_t)(floor(gMeasUnixTime_msec/1000.0)) * 1000));

					printf("Cell ID got in RIC Indication Header: %s\n", gCellID);
					
					bool flag_ue = false; // this is indicator for writing data into influxDB, that is to send data on socket
					bool flag_cell = false; // this is indicator for writing data into influxDB, that is to send data on socket
					UeMetricsEntry *ueMetrics = NULL;
					CellMetricsEntry *cellMetrics = NULL;

					int payload_size = ricIndication->protocolIEs.list.array[idx]-> \
																		 value.choice.RICindicationMessage.size;


					char* payload = (char*) calloc(payload_size, sizeof(char));
					memcpy(payload, ricIndication->protocolIEs.list.array[idx]-> \
																		 value.choice.RICindicationMessage.buf, payload_size);

					//printf("Payload %s\n", payload);
					printf("======== RIC indication message =========\n");
					DumpHex(payload, payload_size);

					// E2SM-KPM Details
					// 1. add some header files and c files into asn1c_defs: move all files in oran-e2sim/src/ASN1c into xapp-sm-connector/asn1c_defs
					// 2. delete or comment "...asn_oer_...." parts .
					// 1-1) header files: E2SM-KPM-IndicationMessage.h, E2SM-KPM-IndicationMessage-Format1.h, CellObjectID.h
					//      PM-Containers-Item.h, PM-Info-Item.h, PerUE-PM-Item.h, UE-Identity.h, RAN-Container.h, PF-Container.h, ODU-PF-Container.h,
					//      OCUCP-PF-Container.h, OCUUP-PF-Container.h, CellResourceReportListItem.h, NRCGI.h, ServedPlmnPerCellListItem.h,
					//      MeasurementType.h, MeasurementValue.h, MeasurementTypeName.h, MeasurementTypeID.h, NativeReal.h, NRCellIdentity.h
					//      PF-ContainerListItem.h, NI-Type.h, CUUPMeasurement-Container.h, L3-RRC-Measurements.h, RRCEvent.h, PlmnID-Item.h
					//      FGC-DU-PM-Container.h, EPC-DU-PM-Container.h, SlicePerPlmnPerCellListItem.h, SNSSAI.h, PerQCIResportListItem.h, QCI.h,
					//      FGC-CUUP-PM-Format.h, EPC-CUUP-PM-Format.h, ServingCellMeasurements.h, MeasResultNeighCells.h, REAL.h, PerQCIReportListItemFormat.h,
					//      S-NSSAI.h, MeasResultServMOList.h, MeasResultPCell.h, PhysCellId.h, RSRP-Range.h, RSRQ-Range.h, MeasResultListNR.h, MeasResultListEUTRA.h,
					//      FQIPERSlicesPerPlmnPerCellListItem.h, FiveQI.h, SliceToReportListItem.h, MeasResultEUTRA.h, MeasQuantityResultsEUTRA.h, MeasResultNR.h
					//      FQIPERSlicePerPlmnListItem.h, MeasQualityResult.h, SINR-Range.h, RSRP-RangeEUTRA.h, RSRQ-RangeEUTRA.h, SINR-RangeEUTRA.h,
					//      ResultsPerCSI-RS-Index.h, ResultsPerCSI-RS-IndexList.h, ResultsPerSSB-Index.h, ResultsPerSSB-IndexList.h, CSI-RS-Index.h, SSB-Index.h,
					//      MeasResultServMO.h, ServCellIOndex.h
					// 1-2) c files: E2SM-KPM-IndicationMessage.c, E2SM-KPM-IndicationMessage-Format1.c, CellObjectID.c, OCUCP-PF-Container.c, OCUUP-PF-Container.c
					//      PM-Containers-Item.c, PM-Info-Item.c, PerUE-PM-Item.c, UE-Identity.c, RAN-Container.c, PF-Container.c, ODU-PF-Container.c, 
					//      CellResourceReportListItem.c, MeasurementType.c, MeasurementTypeID.c,  MeasurementValue.c, NRCGI.c, ServedPlmnPerCellListItem.c
					//      MeasurementTypeName.c, CUUPMeasurement-Container.c, RRCEvent.c, FGC-DU-PM-Container.c, EPC-DU-PM-Container.c, PlmnID-Item.c,
					//      EPC-CUUP-PM-Format.c, NativeReal.c, L3-RRC-Measurements.c, NRCellIdentity.c, REAL.c, PerQCIReportListItemFormat.c, QCI.c, ServingCellMeasurement.c,
					//      MeasResultNeighCells.c, SlicePerPlmnPerCellListItem.c, SlicePerPlmnPerCellListItem.c, FGC-CUUP-PM-Format.c, PF-ContainerListItem.c
					//      FQIPERSlicesPerPlmnPerCellListItem.c, MeasResultListEUTRA.c, MeasResultEUTRA.c, MeasQuantityResultsEUTRA.c, MeasResultListNR.c, MeasResultNR.c
					//      PerQCIReportListItem.c, RSRP-Range.c, RSRQ-Range.c, S-NSSAI.c, SlicoReportListItem.c, SNSSAI.c, FiveQI.c
					//      FQIPERSlicePerPlmnListItem.c,  MeasQualityResult.c, SINR-Range.c, RSRP-RangeEUTRA.c, RSRQ-RangeEUTRA.c, SINR-RangeEUTRA.c
					//      ResultsPerCSI-RS-Index.c, ResultsPerCSI-RS-IndexList.c, ResultsPerSSB-Index.c, ResultsPerSSB-IndexList.c, SSB-Index.c, PhyCellId.c, 
					//      MeasResultPCell.c, MeasResultServMOList.c, MeasResultServMO.c, ServCellIOndex.c
					// 1-3) delete (comment) and make zero "asn_OER_.... " part in E2SM-KPM-IndicationMessage.c, E2SM-KPM-IndicationMessage-Format1.c, NI-Type.c
					//      CellObjectID.h, UE-Identity.h, RAN-Container.h, MeasurementTypeName.h, MeasurementTypeID.h, NativeReal.h, NRCellIdentity.h
					//      NI-Type.h, RRCEvent.h, QCI.h, REAL.h, PhysCellId.h, RSRP-Range.h, RSRQ-Range.h, FiveQI.h, SINR-Range.h, RSRP-RangeEUTRA.h, RSRQ-RangeEUTRA.h, SINR-RangeEUTRA.h
					//      ,  CSI-RS-Index.h, SSB-Index.h, ServCellIOndex.h
					//
					//      CellObjectID.c, PerUE-PM-Item.c, PF-Container.c, ODU-PF-Container.c, CellResourceReportListItem.c, MeasurementType.c, 
					//      MeasurementValue.c, OCUCP-PF-Container.c, OCUUP-PF-Container.c, MeasurementTypeID.c, MeasurementTypeName.c, CUUPMeasurement-Container.c
					//      RRCEvent.c, FGC-DU-PM-Container.c, EPC-DU-PM-Container.c, EPC-CUUP-PM-Format.c, NRCellIdentity.c, PerQCIReportListItemFormat.c, QCI.c, 
					//      ServingCellMeasurement.c, MeasResultNeighCells.c, SlicePerPlmnPerCellListItem.c, FGC-CUUP-PM-Format.c, FQIPERSlicesPerPlmnPerCellListItem.c
					//      MeasResultListEUTRA.c, MeasResultListNR.c, NI-Type.c, erQCIReportListItem.c, RSRP-Range.c, RSRQ-Range.c, S-NSSAI.c, SlicoReportListItem.c, SNSSAI.c,
					//      FiveQI.c, FQIPERSlicePerPlmnListItem.c, SINR-Range.c, RSRP-RangeEUTRA.c, RSRQ-RangeEUTRA.c, SINR-RangeEUTRA.c, ResultsPerCSI-RS-IndexList.c, 
					//      ResultsPerSSB-IndexList.c,  CSI-RS-Index.c, SSB-Index.c, hyCellId.c, MeasResultServMOList.c, ServCellIOndex.c
					// 1-4) existing file modified: about "... ans_oer_..."
				    //      NativeEnumerated.h, NativeEnumerted.c

					E2SM_KPM_IndicationMessage_t* decodedMsg = e2sm_decode_ric_indication_message(payload, payload_size);
					//printf("// E2SM-KPM decodedMsg= %x\n",decodedMsg);

					unsigned long indMsg_present = (unsigned long) (decodedMsg->present);
					printf("Ind Msg present: %ld\n", indMsg_present);
					if (indMsg_present == 1) {
						//printf("////entered indMsg_present = 1 \n");
						E2SM_KPM_IndicationMessage_Format1_t *indMsgrFormat1 = decodedMsg->choice.indicationMessage_Format1;

						// 1. Processing pm_Containers
						int count_PMContainer = int(indMsgrFormat1->pm_Containers.list.count);
						printf("// --PMContainer counts: %d \n", count_PMContainer);
						for (int i = 0; i < count_PMContainer; i++) { // Always one loop (that is, count_PMContainer == 1)
							// Do I need to initialize metric values??

							printf("// -- -- %d-th PMContainer processing \n", i+1);
							
							PM_Containers_Item_t pmContainer = *(PM_Containers_Item_t *)(indMsgrFormat1->pm_Containers.list.array[i]);
							/* Errors:
							// error 1 : 'PM_Containers_Item_t' was not declared in this scope
							// SOLVED: insert #include <PM-Containers-Item.h> in msgs_proc.hpp
							// error 2: error: invalid use of incomplete type 'struct PF_Container, ....'
							// SOLVED: insert #include <PM-Container.h>, .... in msgs_proc.hpp
							*/
							// 1-1: Processing performanceContainer
							if (pmContainer.performanceContainer != NULL) { // performanceContainer is optional
								//printf("////// entered e2sm if pmContainer.performanceContainer != NULL \n");
								int perfContainerType = (int)(pmContainer.performanceContainer->present);
								gPFContainterType = perfContainerType;
								//printf("////// e2sm perfContainerType: %d\n", perfContainerType);
								if (perfContainerType == 1) {
									printf("// -- -- e2sm in type1 perfContainerType of oDU \n");
									struct ODU_PF_Container	*oDU_PF = pmContainer.performanceContainer->choice.oDU;
									int count_oDUcellResourceReportListItem = oDU_PF->cellResourceReportList.list.count;
									printf("// -- -- oDU - cell Resource Report List Item Count: %d \n", count_oDUcellResourceReportListItem);

									// Cell Metric: PRBUsage (AvailablePRBs)
									if ((gPFContainterType == 1) && (flag_cell == false)) {
										cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));
										flag_cell = true;
										cellMetrics->MetricType = (0xF0F0000000000000 + (1 << (gPFContainterType - 1))); // Cell Metric for O-CUUP (gPFContainterType == 3)
										printf("TEST of bit operator in ODU: 0x%lX\n",(1 << (gPFContainterType - 1)));
										printf("TEST of Metric Type for Cell Metric in ODU: 0x%lX\n",cellMetrics->MetricType);
										cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(cellMetrics->CellID, gCellID, strlen(gCellID));
									}
									//////////
									// Ue Metric: DlThp 
									if ((gPFContainterType == 1) && (flag_ue == false)) {
										ueMetrics = (UeMetricsEntry *) malloc(sizeof(UeMetricsEntry));
										flag_ue = true;
										ueMetrics->MetricType = (0x1010000000000000 + (1 << (gPFContainterType - 1))); // Cell Metric for O-CUUP (gPFContainterType == 3)
										printf("TEST of bit operator in ODU: 0x%lX\n",(1 << (gPFContainterType - 1)));
										printf("TEST of Metric Type for UE Metric in ODU: 0x%lX\n",ueMetrics->MetricType);
										ueMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(ueMetrics->ServingCellID, gCellID, strlen(gCellID));
									}
									//////////
									for (int j = 0; j < count_oDUcellResourceReportListItem; j++) {
										printf("%d-th oDU - cell Resource Report List Item processing \n", j+1);
										
										CellResourceReportListItem_t cellResourceReport = *(CellResourceReportListItem_t *)(oDU_PF->cellResourceReportList.list.array[j]);
										long plmnid_size = (long)(cellResourceReport.nRCGI.pLMN_Identity.size);
										//printf("oDU - cell Resource Report: NRCGI pLMN_Identity ---\n");
										//DumpHex((char *)(cellResourceReport.nRCGI.pLMN_Identity.buf), plmnid_size);

										char *test_plmnId = ParsePLMNIdentity(cellResourceReport.nRCGI.pLMN_Identity.buf, plmnid_size);
										if (test_plmnId != NULL) {
											printf("PLMNIdentity value %s\n", test_plmnId);
										}

										long nrCellID_size = (long)(cellResourceReport.nRCGI.nRCellIdentity.size);
										int nrCellID_BitsUnsued_size = (int)(cellResourceReport.nRCGI.nRCellIdentity.bits_unused);
										//printf("--- oDU - cell Resource Report: NRCGI nRCellIdentity (UnUsed bits %d)---\n", nrCellID_BitsUnsued_size);
										//DumpHex((char *)(cellResourceReport.nRCGI.nRCellIdentity.buf), nrCellID_size);

										char *test_CellID = ParseNRCGI(cellResourceReport.nRCGI);
										if (test_CellID != NULL) {
											printf("NRCGI value %s\n", test_CellID);
										}

										long stat_dl_TotalofAvailablePRBs =  -1;
										if (cellResourceReport.dl_TotalofAvailablePRBs != NULL) {
											stat_dl_TotalofAvailablePRBs =  (long)(*(cellResourceReport.dl_TotalofAvailablePRBs));
											printf("dl_TotalofAvailablePRBs : %ld\n", stat_dl_TotalofAvailablePRBs);
										} 
										// Cell Metric for O-DU
										if ((gPFContainterType == 1) && flag_cell) {
											cellMetrics->TotDLAvailPRBs = stat_dl_TotalofAvailablePRBs;
										}
										//////////

										long stat_ul_TotalofAvailablePRBs =  -1;
										if (cellResourceReport.ul_TotalofAvailablePRBs != NULL) {
											stat_ul_TotalofAvailablePRBs =  (long)(*(cellResourceReport.ul_TotalofAvailablePRBs));
											printf("ul_TotalofAvailablePRBs : %ld\n", stat_ul_TotalofAvailablePRBs);
										} 
										//cellMetrics.TotalofAvailablePRBsUL = stat_ul_TotalofAvailablePRBs;
										//cellMetrics.MeasPeriodPDCP = 20; // why?

										int count_ServedPlmnPerCellListItem = cellResourceReport.servedPlmnPerCellList.list.count;
										for (int k = 0; k < count_ServedPlmnPerCellListItem; k++) {
											ServedPlmnPerCellListItem_t servedPlmnPerCell = (ServedPlmnPerCellListItem_t)(*(cellResourceReport.servedPlmnPerCellList.list.array[k]));
											long served_plmnid_size = (long)(servedPlmnPerCell.pLMN_Identity.size);
											printf("--- oDU - cell Resource Report: Served pLMN_Identity ---\n");
											//DumpHex((char *)(servedPlmnPerCell.pLMN_Identity.buf), served_plmnid_size);

											if (servedPlmnPerCell.du_PM_5GC != NULL) {
												printf("enterend servedPlmnPerCell.du_PM_5GC \n");
												int count_slicePerPlmnPerCellListItem = (int)(servedPlmnPerCell.du_PM_5GC->slicePerPlmnPerCellList.list.count);
												for (int l = 0; l < count_slicePerPlmnPerCellListItem; l++) {
													printf("%d-th slice per plmn per cell list item \n", l);
													SlicePerPlmnPerCellListItem_t slicePerPlmnPerCell = *(SlicePerPlmnPerCellListItem_t *)(servedPlmnPerCell.du_PM_5GC->slicePerPlmnPerCellList.list.array[l]);
													int sliceID_sST_size = slicePerPlmnPerCell.sliceID.sST.size;
													printf("--- slice ID sST in slice per plmn per cell ---\n");
													DumpHex((char *)(slicePerPlmnPerCell.sliceID.sST.buf), sliceID_sST_size);

													if (slicePerPlmnPerCell.sliceID.sD != NULL) {
														int sliceID_sD_size = slicePerPlmnPerCell.sliceID.sD->size;
														printf("--- slice ID sD in slice per plmn per cell ---\n");
														DumpHex((char *)(slicePerPlmnPerCell.sliceID.sD->buf), sliceID_sD_size);
													}

													int count_FQIPERSlicesPerPlmnPerCell = int(slicePerPlmnPerCell.fQIPERSlicesPerPlmnPerCellList.list.count);
													for (int m = 0; m < count_FQIPERSlicesPerPlmnPerCell; m++) {
														printf("%d-th FQI per slice per plmn per cell list item \n", m);
														FQIPERSlicesPerPlmnPerCellListItem_t fQIPerSlicePerPlmnPerCell = *(FQIPERSlicesPerPlmnPerCellListItem_t *)(slicePerPlmnPerCell.fQIPERSlicesPerPlmnPerCellList.list.array[m]);
														long val_fiveQI = fQIPerSlicePerPlmnPerCell.fiveQI;
														printf("----------- val fiveQI : %ld \n", val_fiveQI);
														//ueMetrics.FiveQI = val_fiveQI;

														long val_FQI_dl_PRBUsage = -1;
														if (fQIPerSlicePerPlmnPerCell.dl_PRBUsage != NULL) {
															val_FQI_dl_PRBUsage = (long)(*(fQIPerSlicePerPlmnPerCell.dl_PRBUsage));
														} 
														printf("----------- val FQI_dl_PRBUsage : %ld \n", val_FQI_dl_PRBUsage);
														//ueMetrics.PRBUsageDL = val_FQI_dl_PRBUsage;

														long val_FQI_ul_PRBUsage = -1;
														if (fQIPerSlicePerPlmnPerCell.ul_PRBUsage != NULL) {
															val_FQI_ul_PRBUsage = (long)(*(fQIPerSlicePerPlmnPerCell.ul_PRBUsage));
														} 
														printf("----------- val FQI_ul_PRBUsage : %ld \n", val_FQI_ul_PRBUsage);
														//ueMetrics.PRBUsageUL = val_FQI_ul_PRBUsage;
													}
												}
											}

											if (servedPlmnPerCell.du_PM_EPC != NULL) {
												printf("--- enterend servedPlmnPerCell.du_PM_EPC \n");
												int count_perQCIReportListItem = (int)(servedPlmnPerCell.du_PM_EPC->perQCIReportList_du.list.count);
												for (int l = 0; l < count_perQCIReportListItem; l++) {
													printf("%d-th perQCIReportList item \n", l+1);
													PerQCIReportListItem_t perQCIReport = *(PerQCIReportListItem_t *)(servedPlmnPerCell.du_PM_EPC->perQCIReportList_du.list.array[l]);
													long val_qci = (long) (perQCIReport.qci);
													printf("----------- val_qci : %ld \n", val_qci);
													//ueMetrics.QCI = val_qci;

													long val_perQCIReport_dl_PRBUsage = -1;
													if (perQCIReport.dl_PRBUsage != NULL) {
														val_perQCIReport_dl_PRBUsage = (long)(*perQCIReport.dl_PRBUsage);
														printf("----------- val perQCI_dl_PRBUsage : %ld \n", val_perQCIReport_dl_PRBUsage);
													} 
													//printf("----------- val perQCI_dl_PRBUsage : %ld \n", val_perQCIReport_dl_PRBUsage);
													//ueMetrics.PRBUsageDL =  val_perQCIReport_dl_PRBUsage;

													long val_perQCIReport_ul_PRBUsage = -1;
													if (perQCIReport.ul_PRBUsage != NULL) {
														val_perQCIReport_ul_PRBUsage = (long)(*perQCIReport.ul_PRBUsage);
														printf("----------- val perQCI_ul_PRBUsage : %ld \n", val_perQCIReport_ul_PRBUsage);
													} 
												}

											}
										}
									}
								} else if (perfContainerType == 2) {
									// Ue Metric 
									if (!flag_ue && (strcmp(gCellID, "1111") != 0)) {
										ueMetrics = (UeMetricsEntry *) malloc(sizeof(UeMetricsEntry));
										flag_ue = true;
										ueMetrics->MetricType = (0x1010000000000000 + (1 << (gPFContainterType - 1))); // Ue Metric for O-CUCP (gPFContainterType == 2)
										printf("TEST of bit operator in OCUCP: 0x%lX\n",(1 << (gPFContainterType - 1)));
										printf("TEST of Metric Type for Ue Metric in OCUCP: 0x%lX\n",ueMetrics->MetricType);
										ueMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(ueMetrics->ServingCellID, gCellID, strlen(gCellID));
										ueMetrics->ServingCellID[strlen(gCellID)] = '\0';
									}
									////////
									printf("// -- --  e2sm in type2 perfContainerType of oCU_CP \n");
									struct OCUCP_PF_Container	*oCUCP_PF = pmContainer.performanceContainer->choice.oCU_CP;
									long nActUEs = -1;
									if (oCUCP_PF->cu_CP_Resource_Status.numberOfActive_UEs != NULL) {
										nActUEs = *(long *)(oCUCP_PF->cu_CP_Resource_Status.numberOfActive_UEs);
										printf("--- oCUCP_PF - number of Active UEs: %ld \n", nActUEs);
									} 
									//printf("oCUCP_PF - number of Active UEs: %ld \n", nActUEs);

								} else if (perfContainerType == 3) {
									// Cell Metric
									if (!flag_cell) {
										cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));
										flag_cell = true;
										cellMetrics->MetricType = (0xF0F0000000000000 + (1 << (gPFContainterType - 1))); // Cell Metric for O-CUUP (gPFContainterType == 3)
										printf("TEST of bit operator in OCUUP: 0x%lX\n",(1 << (gPFContainterType - 1)));
										printf("TEST of Metric Type for Cell Metric in OCUUP: 0x%lX\n",cellMetrics->MetricType);
										cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
										strncpy(cellMetrics->CellID, gCellID, strlen(gCellID));
										cellMetrics->CellID[strlen(gCellID)] = '\0';
									}
									////////

									printf("// -- -- e2sm in type3 perfContainerType of oCU_UP \n");
									struct OCUUP_PF_Container	*oCUUP_PF = pmContainer.performanceContainer->choice.oCU_UP;
									int count_oCUUPpfContainerListItem = oCUUP_PF->pf_ContainerList.list.count;
									printf("--- oCUUP - PF Container List Item Count: %d \n", count_oCUUPpfContainerListItem);
									for (int j = 0; j < count_oCUUPpfContainerListItem; j++) {
										printf("%d-th oCUUP - PF Container List Item processing \n", j+1);
										
										PF_ContainerListItem_t oCUUP_PF_item =  *(PF_ContainerListItem_t *)(oCUUP_PF->pf_ContainerList.list.array[j]);
										long val_oCUUP_PF_interface_type = (long) (oCUUP_PF_item.interface_type);
										printf("val. oCUUP PF_interface type: %ld \n", val_oCUUP_PF_interface_type);

										int count_oCUUP_PMContainer_plmnListItem= (int)(oCUUP_PF_item.o_CU_UP_PM_Container.plmnList.list.count);
										for (int k = 0; k < count_oCUUP_PMContainer_plmnListItem; k++) {
											printf("%d-th oCUUP_PMContainer_plmnList item \n", k);
											PlmnID_Item_t cuUPPlmn = *(PlmnID_Item_t *)(oCUUP_PF_item.o_CU_UP_PM_Container.plmnList.list.array[k]);
											int cuUPPlmn_pLMN_ID_size = (int)(cuUPPlmn.pLMN_Identity.size);
											printf("--- cuUPPlmn_pLMN_ID ---\n");
											DumpHex((char *)(cuUPPlmn.pLMN_Identity.buf), cuUPPlmn_pLMN_ID_size);

											char *test_cuUPPlmn_pLMN_ID = ParsePLMNIdentity(cuUPPlmn.pLMN_Identity.buf, cuUPPlmn_pLMN_ID_size);
											if (test_cuUPPlmn_pLMN_ID != NULL) {
												printf("test_cuUPPlmn_pLMN_ID %s\n", test_cuUPPlmn_pLMN_ID);
											}

											if (cuUPPlmn.cu_UP_PM_5GC != NULL) {
												printf("enterend cuUPPlmn.cu_UP_PM_5GC \n");
												int count_sliceToReportListItem = (int)(cuUPPlmn.cu_UP_PM_5GC->sliceToReportList.list.count);
												printf("count_sliceToReportListItem: %d \n", count_sliceToReportListItem);
												for (int l = 0; l < count_sliceToReportListItem; l++) {
													printf("%d-th slice to report list item \n", l);
													SliceToReportListItem_t sliceToReportListItem = *(SliceToReportListItem_t *)(cuUPPlmn.cu_UP_PM_5GC->sliceToReportList.list.array[l]);
													
													int sliceID_sST_size2 = sliceToReportListItem.sliceID.sST.size;
													printf("--- slice ID sST in slice to report ---\n");
													DumpHex((char *)(sliceToReportListItem.sliceID.sST.buf), sliceID_sST_size2);
													if (sliceToReportListItem.sliceID.sD != NULL) {
														int sliceID_sD_size2 = sliceToReportListItem.sliceID.sD->size;
														printf("--- slice ID sD in  slice to report  ---\n");
														DumpHex((char *)(sliceToReportListItem.sliceID.sD->buf), sliceID_sD_size2);
													}
													
													int count_FQIPERSlicesPerPlmnList = int(sliceToReportListItem.fQIPERSlicesPerPlmnList.list.count);
													printf("count_FQIPERSlicesPerPlmnList: %d \n", count_FQIPERSlicesPerPlmnList);
													for (int m = 0; m < count_FQIPERSlicesPerPlmnList; m++) {
														//UeMetricsEntry ueMetrics;

														printf("%d-th FQI per slice per plmn list item \n", m);
														FQIPERSlicesPerPlmnListItem_t fQIPerSlicePerPlmn = *(FQIPERSlicesPerPlmnListItem_t *)(sliceToReportListItem.fQIPERSlicesPerPlmnList.list.array[m]);
														
														long val_fiveQI2 = fQIPerSlicePerPlmn.fiveQI;
														printf("----------- val fiveQI2 : %ld \n", val_fiveQI2);

														if (fQIPerSlicePerPlmn.pDCPBytesDL != NULL) {
															int size_FQI_pDCPBytesDL = (int)(fQIPerSlicePerPlmn.pDCPBytesDL->size);
															printf("--- FQI_pDCPBytesDL  ---\n");
															DumpHex((char *)(fQIPerSlicePerPlmn.pDCPBytesDL->buf), size_FQI_pDCPBytesDL);
														} 
														if (fQIPerSlicePerPlmn.pDCPBytesUL != NULL) {
															int size_FQI_pDCPBytesUL = (int)(fQIPerSlicePerPlmn.pDCPBytesUL->size);
															printf("--- FQI_pDCPBytesUL  ---\n");
															DumpHex((char *)(fQIPerSlicePerPlmn.pDCPBytesUL->buf), size_FQI_pDCPBytesUL);
														} 
													}
												}
											}

											if (cuUPPlmn.cu_UP_PM_EPC != NULL) {
												printf("enterend cuUPPlmn.cu_UP_PM_EPC \n");
												int count_perQCIReportList_cuup_Item = (int)(cuUPPlmn.cu_UP_PM_EPC->perQCIReportList_cuup.list.count);
												printf("count_perQCIReportList_cuup_Item: %d \n", count_perQCIReportList_cuup_Item);
												for (int l = 0; l < count_perQCIReportList_cuup_Item; l++) {
													//UeMetricsEntry ueMetrics;

													printf("%d-th perQCIReportList_cuup item \n", l);
													PerQCIReportListItemFormat_t perQCIReport2 = *(PerQCIReportListItemFormat_t *)(cuUPPlmn.cu_UP_PM_EPC->perQCIReportList_cuup.list.array[l]);
													long val_drbqci = (long) (perQCIReport2.drbqci);
													printf("----------- val drbqci  : %ld \n", val_drbqci);
													//ueMetrics.QCI = val_drbqci;

													if (perQCIReport2.pDCPBytesDL != NULL) {
														int size_perQCIReport_pDCPBytesDL = (int)(perQCIReport2.pDCPBytesDL->size);
														printf("--- QCIReport_pDCPBytesDL  --- (size: %d)\n", size_perQCIReport_pDCPBytesDL); // 2 or 3 bytes in case of eNB and 1 byte in case of gNB
														DumpHex((char *)(perQCIReport2.pDCPBytesDL->buf), size_perQCIReport_pDCPBytesDL);
														char chr_pDCPBytesDL[size_perQCIReport_pDCPBytesDL];
														strncpy((char *)chr_pDCPBytesDL, (char *)(perQCIReport2.pDCPBytesDL->buf), size_perQCIReport_pDCPBytesDL);
														if (size_perQCIReport_pDCPBytesDL == 2) { 
															//printf("test size 2- \n");
															//DumpHex((char *)chr_pDCPBytesDL, size_perQCIReport_pDCPBytesDL);
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 8) | ((int)chr_pDCPBytesDL[1]);
															printf("--- val. QCIReport_pDCPBytesDL: %d\n", val_pDCPBytesDL);
														} else if (size_perQCIReport_pDCPBytesDL == 3) { // in case of macro eNB
															//printf("test size 3- \n");
															//DumpHex((char *)chr_pDCPBytesDL, size_perQCIReport_pDCPBytesDL);
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 16) | (((int)chr_pDCPBytesDL[1]) << 8) | ((int)chr_pDCPBytesDL[2]);
															printf("--- val. QCIReport_pDCPBytesDL: %d\n", val_pDCPBytesDL);
														} else if (size_perQCIReport_pDCPBytesDL == 4) { // in case of macro eNB
															//printf("test size 4 - \n");
															//DumpHex((char *)chr_pDCPBytesDL, size_perQCIReport_pDCPBytesDL);
															int val_pDCPBytesDL = (((int)chr_pDCPBytesDL[0]) << 24) | (((int)chr_pDCPBytesDL[1]) << 16) | (((int)chr_pDCPBytesDL[2]) << 8) | ((int)chr_pDCPBytesDL[3]);
															printf("--- val. QCIReport_pDCPBytesDL: %d\n", val_pDCPBytesDL);
														}
														//strncpy(ueMetrics.pDCPBytesDL, (char *)(perQCIReport2.pDCPBytesDL->buf), size_perQCIReport_pDCPBytesDL);
													} 
													if (perQCIReport2.pDCPBytesUL != NULL) {
														int size_perQCIReport_pDCPBytesUL = (int)(perQCIReport2.pDCPBytesUL->size);
														printf("--- QCIReport_pDCPBytesUL  --- (size: %d)\n", size_perQCIReport_pDCPBytesUL);
														DumpHex((char *)(perQCIReport2.pDCPBytesUL->buf), size_perQCIReport_pDCPBytesUL);
														//strncpy(ueMetrics.pDCPBytesUL, (char *)(perQCIReport2.pDCPBytesUL->buf), size_perQCIReport_pDCPBytesUL);
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
								printf("--- theRANContainer ---\n");
								DumpHex((char *)(pmContainer.theRANContainer->buf), theRANContainer_size);
							}
						}

						// 2. Processing cellObjectID - oDU (cellID): 4 bytes, oCUUP: 0bytes, oCUCP(NRCellCU): 8 bytes
						long size_cellObjectID = (long) (indMsgrFormat1->cellObjectID.size);
						printf("// -- cellObjectID --- (size: %ld)\n", size_cellObjectID);
						if (size_cellObjectID > 0) {
							DumpHex((char *)(indMsgrFormat1->cellObjectID.buf), size_cellObjectID);
						}
						
						// 3. Processing *list_of_PM_Information
						
						if (indMsgrFormat1->list_of_PM_Information != NULL) {
							int count_PMInfo = int(indMsgrFormat1->list_of_PM_Information->list.count);
							printf("PMInformation counts: %d \n", count_PMInfo);
							char tempMeasurementTypeName[30] = ""; // This is for RRU.PrbUsedDl of Cell Metric in O-DU
							long temp_PrbUsedDl = -1;

							for (int i = 0; i < count_PMInfo; i++) {
								printf("%d-th PMInformation processing \n", i);
								PM_Info_Item_t pmInfo = *(PM_Info_Item_t *)(indMsgrFormat1->list_of_PM_Information->list.array[i]);
								// 3-1: pmType
								int pmInfo_type = (int)(pmInfo.pmType.present);
								if (pmInfo_type == 1) { // MeasurementType_PR_measName
									printf("// -- -- MeasurementType_PR_measName \n");
									MeasurementTypeName_t val_measName = pmInfo.pmType.choice.measName;
									int val_measName_size = (int) (val_measName.size);
									printf("MeasurementTypeName (size: %d)\n", val_measName_size);
									DumpHex((char *)(val_measName.buf), val_measName_size);
									strncpy(tempMeasurementTypeName, (char *)(val_measName.buf), val_measName_size);
									tempMeasurementTypeName[val_measName_size] = '\0';
									
								} else if (pmInfo_type == 2) { // MeasurementType_PR_measID
									printf("// -- -- MeasurementType_PR_measID \n");
									long val_measID = (long)(pmInfo.pmType.choice.measID);
									printf("MeasurementType_PR_measID: %ld\n", val_measID);
								} else {
									printf("Invalid pmInfo type \n");
								}

								// 3-2: pmVal
								int pmVal_type = (int)(pmInfo.pmVal.present);
								if (pmVal_type == 1) { // long	 valueInt
									long pmVal_valueInt = (long)(pmInfo.pmVal.choice.valueInt);
									printf("pmVal_valueInt: %ld\n", pmVal_valueInt);
									temp_PrbUsedDl = pmVal_valueInt;
								} else if (pmVal_type == 2) { // double	 valueReal
									double pmVal_valueReal = (double)(pmInfo.pmVal.choice.valueReal);
									printf("pmVal_valueReal: %f\n", pmVal_valueReal);
								} else if (pmVal_type == 3) { // no value

								} else if (pmVal_type == 4) { // struct L3_RRC_Measurements	*valueRRC
									L3_RRC_Measurements_t rrcValue = *(L3_RRC_Measurements_t *)(pmInfo.pmVal.choice.valueRRC);
									long val_rrcEvent = (long)(rrcValue.rrcEvent);
									if (val_rrcEvent == 0) {
										printf("RRCEvent_b1 with val_rrcEvent = 0 \n");
									} else if (val_rrcEvent == 1) {
										printf("RRCEvent_a3 with val_rrcEvent = 1 \n");
									} else if (val_rrcEvent == 2) {
										printf("RRCEvent_a5 with val_rrcEvent = 2 \n");
									} else if (val_rrcEvent == 3) {
										printf("RRCEvent_periodic with val_rrcEvent = 3 \n");
									} else {
										printf("unsupported rrcEvent vlaue \n");
									}

									if (rrcValue.servingCellMeasurements != NULL) {
										int servCellMeasType = (int)(rrcValue.servingCellMeasurements->present);
										if (servCellMeasType == 1) { // struct MeasResultServMOList	*nr_measResultServingMOList;
											printf("Serving Cell Measurements Type: nr_measResultServingMOList \n");
											int count_nr_measResultServMO = (int)(rrcValue.servingCellMeasurements->choice.nr_measResultServingMOList->list.count);
											for (int j = 0; j < count_nr_measResultServMO; j++) {
												printf("%d-th nr_measResultServMO \n", j);
												MeasResultServMO_t nr_measResultServingMO = *(MeasResultServMO_t *)(rrcValue.servingCellMeasurements->choice.nr_measResultServingMOList->list.array[j]);
												long measResServMO_servCellId = (long)(nr_measResultServingMO.servCellId);
												printf("measResServMO_servCellId: %ld", measResServMO_servCellId);

												long val_physCellId = (long)(*(PhysCellId_t *)(nr_measResultServingMO.measResultServingCell.physCellId));
												printf("val_physCellId: %ld \n", val_physCellId);

												if (nr_measResultServingMO.measResultServingCell.measResult.cellResults.resultsSSB_Cell != NULL) {
													struct MeasQuantityResults *val_resultsSSB_Cell = nr_measResultServingMO.measResultServingCell.measResult.cellResults.resultsSSB_Cell;

													long val_resSSBCell_rsrp = -1;
													if (val_resultsSSB_Cell->rsrp != NULL) {
														val_resSSBCell_rsrp = *(long *)(val_resultsSSB_Cell->rsrp);
														printf("val_resSSBCell_rsrp : %ld \n", val_resSSBCell_rsrp);
													}
													//printf("val_resSSBCell_rsrp : %ld \n", val_resSSBCell_rsrp);

													long val_resSSBCell_rsrq = -1;
													if (val_resultsSSB_Cell->rsrq != NULL) {
														val_resSSBCell_rsrq = *(long *)(val_resultsSSB_Cell->rsrq);
														printf("val_resSSBCell_rsrq : %ld \n", val_resSSBCell_rsrq);
													}
													//printf("val_resSSBCell_rsrq : %ld \n", val_resSSBCell_rsrq);

													long val_resSSBCell_sinr = -1;
													if (val_resultsSSB_Cell->sinr != NULL) {
														val_resSSBCell_sinr = *(long *)(val_resultsSSB_Cell->sinr);
														printf("val_resSSBCell_sinr : %ld \n", val_resSSBCell_sinr);
													}
													//printf("val_resSSBCell_sinr : %ld \n", val_resSSBCell_sinr);
												}
												if (nr_measResultServingMO.measResultServingCell.measResult.cellResults.resultsCSI_RS_Cell != NULL) {
													printf("resultsCSI_RS_Cell \n");
												}
												if (nr_measResultServingMO.measResultServingCell.measResult.rsIndexResults->resultsSSB_Indexes != NULL) {
													printf("resultsSSB_Indexes \n");
												}
												if (nr_measResultServingMO.measResultServingCell.measResult.rsIndexResults->resultsCSI_RS_Indexes != NULL) {
													printf("resultsCSI_RS_Indexes \n");
												}
											}
										} if (servCellMeasType == 2) { // struct MeasResultPCell	*eutra_measResultPCell;
											printf("Serving Cell Measurements Type: eutra_measResultPCell \n");

										} else {
											printf("unsupported Serving Cell Measurements Type \n");
										}
									}
								} else {
									printf("Invalid pmValue type \n");
								}
								// Cell Metric: RRU.PrbUsedDl
								printf("TEST (OUT): tempMeasurementTypeName is %s\n", tempMeasurementTypeName);
								if (strncmp(tempMeasurementTypeName, "RRU.PrbUsedDl", strlen("RRU.PrbUsedDl")) == 0) { //MeasurementTypeName == "RRU.PrbUsedDl"
									printf("TEST (IN): tempMeasurementTypeName is %s\n", tempMeasurementTypeName);
									if ((gPFContainterType == 1) && (flag_cell)) {
										cellMetrics->UsedPRBs = temp_PrbUsedDl;
										printf("TEST: value of RRU.PrbUsedDl is %ld\n", temp_PrbUsedDl);
									}
								}
								//////////


							} // for loop END
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

							printf("list_of_matched_UEs counts: %d \n", count_matchedUEs);
							for (int i = 0; i < count_matchedUEs; i++) {
								// Ue Metric: RemainingUEsCount(O), UeID, DL_throughput, ServingCellRF, NeighborCell1/2/3_RF
								if (flag_ue) {
									ueMetrics->RemainingUEsCount = count_matchedUEs - (i + 1);
								}
								///////////////////
								printf("%d-th matched_UE processing \n", i);
								PerUE_PM_Item_t perUE_PM = *(PerUE_PM_Item_t *)(indMsgrFormat1->list_of_matched_UEs->list.array[i]);
								int size_ueID = (int)(perUE_PM.ueId.size);
								printf("PerUE_PM UE Identity (size: %d)\n", size_ueID);
								DumpHex((char *)(perUE_PM.ueId.buf), size_ueID);
								strncpy(strUEID, (char *)(perUE_PM.ueId.buf), size_ueID);
								intUEID = atoi(strUEID);
								printf("TEST: tempUEID = (str) %s, (int) %d\n", strUEID, intUEID);
								// Ue Metric: RemainingUEsCount, UeID(0), DL_throughput, ServingCellRF, NeighborCell1/2/3_RF
								if (flag_ue) {
									ueMetrics->UeID = intUEID;
								}
								///////////////////
								indicatorMapUE = indicatorMapUE | (1 << (intUEID-1));
								printf("TEST: indicatorMapUE is %x\n", indicatorMapUE);

								int count_PerUE_PM_info= int(perUE_PM.list_of_PM_Information->list.count);
								printf("count_PerUE_PM_info: %d \n", count_PerUE_PM_info);
								for (int j = 0; j < count_PerUE_PM_info; j++) {
									printf("%d-th PMInformation processing for perUE_PM \n", j);
									PM_Info_Item_t pmInfo_perUE_PM = *(PM_Info_Item_t *)(perUE_PM.list_of_PM_Information->list.array[j]);

									char ueMeasurementTypeName[30] = ""; // This is for DRB.UEThpDl.UEID of Ue Metric in O-DU
									double temp_DRB_UEThpDl_UEID = -1; // double

									// 4-1: pmType
									int pmInfo_type_perUE_PM = (int)(pmInfo_perUE_PM.pmType.present);
									if (pmInfo_type_perUE_PM == 1) { // MeasurementType_PR_measName
										printf("// -- -- MeasurementType_PR_measName \n");
										MeasurementTypeName_t val_measName_perUE_PM = pmInfo_perUE_PM.pmType.choice.measName;
										int val_measName_size_perUE_PM = (int) (val_measName_perUE_PM.size);
										printf("MeasurementTypeName (size: %d)\n", val_measName_size_perUE_PM);
										DumpHex((char *)(val_measName_perUE_PM.buf), val_measName_size_perUE_PM);

										strncpy(ueMeasurementTypeName, (char *)(val_measName_perUE_PM.buf), val_measName_size_perUE_PM);
										ueMeasurementTypeName[val_measName_size_perUE_PM] = '\0';
										
									} else if (pmInfo_type_perUE_PM == 2) { // MeasurementType_PR_measID
										printf("// -- -- MeasurementType_PR_measID \n");
										long val_measID_perUE_PM = (long)(pmInfo_perUE_PM.pmType.choice.measID);
										printf("MeasurementType_PR_measID: %ld\n", val_measID_perUE_PM);
									} else {
										printf("Invalid pmInfo type \n");
									}

									// 4-2: pmVal
									int pmVal_type_perUE_PM = (int)(pmInfo_perUE_PM.pmVal.present);
									if (pmVal_type_perUE_PM == 1) { // long	 valueInt
										long pmVal_valueInt_perUE_PM = (long)(pmInfo_perUE_PM.pmVal.choice.valueInt);
										printf("pmVal_valueInt: %ld\n", pmVal_valueInt_perUE_PM);
									} else if (pmVal_type_perUE_PM == 2) { // double	 valueReal
										double pmVal_valueReal_perUE_PM = (double)(pmInfo_perUE_PM.pmVal.choice.valueReal);
										printf("pmVal_valueReal: %f\n", pmVal_valueReal_perUE_PM);
										temp_DRB_UEThpDl_UEID = pmVal_valueReal_perUE_PM; // this value may not be DRB_UEThpDl_UEID
									} else if (pmVal_type_perUE_PM == 3) { // no value

									} else if (pmVal_type_perUE_PM == 4) { // struct L3_RRC_Measurements	*valueRRC
										L3_RRC_Measurements_t rrcValue_perUE_PM = *(L3_RRC_Measurements_t *)(pmInfo_perUE_PM.pmVal.choice.valueRRC);
										long val_rrcEvent_perUE_PM = (long)(rrcValue_perUE_PM.rrcEvent);
										if (val_rrcEvent_perUE_PM == 0) {
											printf("RRCEvent_b1 with val_rrcEvent = 0 \n");
										} else if (val_rrcEvent_perUE_PM == 1) {
											printf("RRCEvent_a3 with val_rrcEvent = 1 \n");
										} else if (val_rrcEvent_perUE_PM == 2) {
											printf("RRCEvent_a5 with val_rrcEvent = 2 \n");
										} else if (val_rrcEvent_perUE_PM == 3) {
											printf("RRCEvent_periodic with val_rrcEvent = 3 \n");
										} else {
											printf("unsupported rrcEvent vlaue \n");
										}

										if (rrcValue_perUE_PM.servingCellMeasurements != NULL) {
											printf("p4 - L3 RRC: servingCellMeasurements \n");
											ServingCellMeasurements_t *SCM = (ServingCellMeasurements_t *)(rrcValue_perUE_PM.servingCellMeasurements);
											//int rrc_ServingCellRFellMeas = rrcValue_perUE_PM.servingCellMeasurements->present;
											int rrc_ServingCellRFellMeas = int(SCM->present);
											if (rrc_ServingCellRFellMeas == 1) { //nr_measResultServingMOList
												//int count_nr_measResultServMO1 = (int) (rrcValue_perUE_PM.servingCellMeasurements->choice.nr_measResultServingMOList->list.count);
												int count_nr_measResultServMO1 = int(SCM->choice.nr_measResultServingMOList->list.count);
												printf("count_nr_measResultServMO1 %d \n", count_nr_measResultServMO1);
												for (int k = 0; k < count_nr_measResultServMO1; k++) {
													printf("%d-th nr_measResultServMO \n", k);
													//MeasResultServMO_t nr_measResultServingMO1 = *(MeasResultServMO_t *)(rrcValue_perUE_PM.servingCellMeasurements->choice.nr_measResultServingMOList->list.array[k]);
													MeasResultServMO_t nr_measResultServingMO1 = *(MeasResultServMO_t *)(SCM->choice.nr_measResultServingMOList->list.array[k]);
													long measResServMO_servCellId1 = (long)(nr_measResultServingMO1.servCellId);
													printf("measResServMO_servCellId1: %ld\n", measResServMO_servCellId1);

													long val_physCellId1 = (long)(*(PhysCellId_t *)(nr_measResultServingMO1.measResultServingCell.physCellId));
													printf("val_physCellId: %ld \n", val_physCellId1);

													if (nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsSSB_Cell != NULL) {
														struct MeasQuantityResults *val_resultsSSB_Cell1 = nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsSSB_Cell;

														long val_resSSBCell_rsrp1 = -1;
														if (val_resultsSSB_Cell1->rsrp != NULL) {
															val_resSSBCell_rsrp1 = *(long *)(val_resultsSSB_Cell1->rsrp);
														}
														//printf("val_resSSBCell_rsrp1 : %ld \n", val_resSSBCell_rsrp1);

														long val_resSSBCell_rsrq1 = -1;
														if (val_resultsSSB_Cell1->rsrq != NULL) {
															val_resSSBCell_rsrq1 = *(long *)(val_resultsSSB_Cell1->rsrq);
														}
														//printf("val_resSSBCell_rsrq1 : %ld \n", val_resSSBCell_rsrq1);

														long val_resSSBCell_sinr1 = -1;
														if (val_resultsSSB_Cell1->sinr != NULL) {
															val_resSSBCell_sinr1 = *(long *)(val_resultsSSB_Cell1->sinr);
														}
														printf("val_resSSBCell_sinr1 : %ld \n", val_resSSBCell_sinr1);
														// Ue Metric: RemainingUEsCount, UeID, DL_throughput, ServingCellRF (O), NeighborCell1/2/3_RF
														if ((gPFContainterType == 2) && (flag_ue)) {
															ueMetrics->ServingCellRF.RSSINR = val_resSSBCell_sinr1;
															printf("TEST: value of ServingCellRF RSSINR is %ld\n", val_resSSBCell_sinr1);
														}
														///////////////////
													}
													//printf("HERE 1 (k = %d)\n", k);
													if (nr_measResultServingMO1.measResultServingCell.measResult.cellResults.resultsCSI_RS_Cell != NULL) {
														printf("resultsCSI_RS_Cell1 \n");
													}
													//printf("HERE 2 (k = %d)\n", k);
													/*
													ResultsPerSSB_IndexList_t *AAA = (ResultsPerSSB_IndexList_t *)(nr_measResultServingMO1.measResultServingCell.measResult.rsIndexResults->resultsSSB_Indexes);
													if (AAA != NULL) {
														printf("TO-DO: resultsSSB_Indexes1 \n");
													}
													printf("HERE 3 (k = %d)\n", k);
													ResultsPerCSI_RS_IndexList_t *BBB = (ResultsPerCSI_RS_IndexList_t *)(nr_measResultServingMO1.measResultServingCell.measResult.rsIndexResults->resultsCSI_RS_Indexes);
													if (BBB != NULL) {
														printf("resultsCSI_RS_Indexes1 \n");
													}
													printf("HERE 4 (k = %d)\n", k);
													*/
												}

											} else if (rrc_ServingCellRFellMeas == 2) { // struct MeasResultPCell	*eutra_measResultPCell;
												printf("TO-DO: rrc Serving Cell Measurements Type: eutra_measResultPCell \n");
											} else {
												printf("rrc unsupported Serving Cell Measurements Type \n");
											}
										}

										if (rrcValue_perUE_PM.measResultNeighCells != NULL) {
											printf("p4 - L3 RRC: measResultNeighCells \n");
											MeasResultNeighCells_t *MRNC = (MeasResultNeighCells_t *)(rrcValue_perUE_PM.measResultNeighCells);
											int mRNCType = int(MRNC->present);
											if (mRNCType == 1) { //
												int count_measResultNR = int(MRNC->choice.measResultListNR->list.count);
												printf("count_measResultNR %d \n", count_measResultNR);
												for (int k = 0; k < count_measResultNR; k++) {
													MeasResultNR_t *valMRNC = (MeasResultNR_t *)(MRNC->choice.measResultListNR->list.array[k]);
													printf("Done: (%d-th) count_measResultNR loops in measResultNeighCells (Type 1) \n", k);
													uint64_t val_physCellId2 = (uint64_t)(*(PhysCellId_t *)(valMRNC->physCellId));
													printf("val_physCellId2: %lu \n", val_physCellId2);

													// Ue Metric: RemainingUEsCount, UeID, DL_throughput, ServingCellRF, NeighborCell1/2/3_RF (Cell ID: 0)
													if ((gPFContainterType == 2) && (flag_ue)) {
														if (k == 0) {
															ueMetrics->NeighborCell1_RF.CellID = val_physCellId2;
															printf("TEST: value of %d-th Neigh. Cell ID is %lu\n", k+1, val_physCellId2);
														} else if (k == 1) {
															ueMetrics->NeighborCell2_RF.CellID = val_physCellId2;
															printf("TTEST: value of %d-th Neigh. Cell ID is %lu\n", k+1, val_physCellId2);
														} else if (k == 2) {
															ueMetrics->NeighborCell3_RF.CellID = val_physCellId2;
															printf("TEST: value of %d-th Neigh. Cell ID is %lu\n", k+1, val_physCellId2);
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
															printf("val_resSSBCell_rsrp2 : %ld \n", val_resSSBCell_rsrp2);
														}
														//printf("val_resSSBCell_rsrp2 : %ld \n", val_resSSBCell_rsrp2);

														long val_resSSBCell_rsrq2= -1;
														if (val_resultsSSB_Cell2->rsrq != NULL) {
															val_resSSBCell_rsrq2 = *(long *)(val_resultsSSB_Cell2->rsrq);
															printf("val_resSSBCell_rsrq2 : %ld \n", val_resSSBCell_rsrq2);
														}
														//printf("val_resSSBCell_rsrq2 : %ld \n", val_resSSBCell_rsrq2);

														long val_resSSBCell_sinr2 = -1;
														if (val_resultsSSB_Cell2->sinr != NULL) {
															val_resSSBCell_sinr2 = *(long *)(val_resultsSSB_Cell2->sinr);
															printf("val_resSSBCell_sinr2 : %ld \n", val_resSSBCell_sinr2);
														}
														//printf("val_resSSBCell_sinr2 : %ld \n", val_resSSBCell_sinr2);
														// Ue Metric: RemainingUEsCount, UeID, DL_throughput, ServingCellRF, NeighborCell1/2/3_RF (Cell ID: 0)
														if ((gPFContainterType == 2) && (flag_ue)) {
															if (k == 0) {
																ueMetrics->NeighborCell1_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
																printf("TEST: value of %d-th Neigh. Cell RSSINR is %lu\n", k+1, val_resSSBCell_sinr2);
															} else if (k == 1) {
																ueMetrics->NeighborCell2_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
																printf("TTEST: value of %d-th Neigh. Cell RSSINR is %lu\n", k+1, val_resSSBCell_sinr2);
															} else if (k == 2) {
																ueMetrics->NeighborCell3_RF.CellRF.RSSINR = val_resSSBCell_sinr2;
																printf("TEST: value of %d-th Neigh. Cell RSSINR is %lu\n", k+1, val_resSSBCell_sinr2);
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
									// Ue Metric: RemainingUEsCount, UeID, DL_throughput (0) : in DU, ServingCellRF, NeighborCell1/2/3_RF
									if (strncmp(ueMeasurementTypeName, "DRB.UEThpDl.UEID", strlen("DRB.UEThpDl.UEID")) == 0) { //MeasurementTypeName == "DRB.UEThpDl.UEID	"
										if ((gPFContainterType == 1) && (flag_ue)) {
											ueMetrics->DL_throughput = temp_DRB_UEThpDl_UEID;
											printf("TEST: value of DRB.UEThpDl.UEID is %f\n", temp_DRB_UEThpDl_UEID);
										}
									}
									///////////////////
								}  
								// Ue Metric: Sending the completed Ue Metric for a specific UE over socket
								if (flag_ue) { // O-CU-CP or O-DU
									printf("======== ueMetric in O-CU-CP is about to be sent over socket  =========\n");
									printf("UeMetric - MetricType: 0x%lX\n", ueMetrics->MetricType);
									printf("UeMetric - MeasUnixTime_msec: %lu\n", ueMetrics->MeasUnixTime_msec);
									printf("UeMetric - MatchedUEsTotNum: %d\n", ueMetrics->MatchedUEsTotNum);
									printf("UeMetric - RemainingUEsCount: %d\n", ueMetrics->RemainingUEsCount);
									printf("UeMetric - UeID: %lu\n", ueMetrics->UeID);
									printf("UeMetric - DlThp (effective only in O-DU): %f\n", ueMetrics->DL_throughput);
									printf("UeMetric - svcID : %s\n", ueMetrics->ServingCellID);
									//printf("UeMetric - svcRF-RSRP : %ld\n", ueMetrics->ServingCellRF.RSRP);
									//printf("UeMetric - svcRF-RSRQ : %ld\n", ueMetrics->ServingCellRF.RSRQ);
									printf("UeMetric - svcRF-RSSINR : %ld\n", ueMetrics->ServingCellRF.RSSINR);
									printf("UeMetric - ngcID1 : %lu\n", ueMetrics->NeighborCell1_RF.CellID);
									//printf("UeMetric - ngcRF1-RSRP : %ld\n", ueMetrics->NeighborCell1_RF.CellRF.RSRP);
									//printf("UeMetric - ngcRF1-RSRQ : %ld\n", ueMetrics->NeighborCell1_RF.CellRF.RSRQ);
									printf("UeMetric - ngcRF1-RSSINR : %ld\n", ueMetrics->NeighborCell1_RF.CellRF.RSSINR);
									printf("UeMetric - ngcID2 : %lu\n", ueMetrics->NeighborCell2_RF.CellID);
									//printf("UeMetric - ngcRF2-RSRP : %ld\n", ueMetrics->NeighborCell2_RF.CellRF.RSRP);
									//printf("UeMetric - ngcRF2-RSRQ : %ld\n", ueMetrics->NeighborCell2_RF.CellRF.RSRQ);
									printf("UeMetric - ngcRF2-RSSINR : %ld\n", ueMetrics->NeighborCell2_RF.CellRF.RSSINR);
									printf("UeMetric - ngcID3 : %lu\n", ueMetrics->NeighborCell3_RF.CellID);
									//printf("UeMetric - ngcRF3-RSRP : %ld\n", ueMetrics->NeighborCell3_RF.CellRF.RSRP);
									//printf("UeMetric - ngcRF3-RSRQ : %ld\n", ueMetrics->NeighborCell3_RF.CellRF.RSRQ);
									printf("UeMetric - ngcRF3-RSSINR : %ld\n", ueMetrics->NeighborCell3_RF.CellRF.RSSINR);
									
									printf("TEST: UeMetric Size to send: %d\n", sizeof(UeMetricsEntry));
									mysend_socket((char *)ueMetrics, agent_ip, sizeof(UeMetricsEntry));

									if (ueMetrics->RemainingUEsCount > 0) {
										printf("TEST: another ue Metric shoud be delivered over socket\n");
									} else if (ueMetrics->RemainingUEsCount == 0) {
										flag_ue = false;
										free(ueMetrics);
									} 
								}
								//////////////////

							}
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
					
					// send payload to agent --> SEHONG: send UE and Cell Metrics to agent. how?
					//std::string agent_ip = find_agent_ip_from_gnb(gnb_id); <-- move it to front part of this case
					//mysend_socket(payload, agent_ip, payload_size);

					// test - Cell Metrics
					// CellMetricsEntry *cellMetrics = (CellMetricsEntry *) malloc(sizeof(CellMetricsEntry));

					// cellMetrics->MetricType = 0xF0F0F0F0F0F0F0F0;
					// cellMetrics->MeasUnixTime_msec = gMeasUnixTime_msec;
					// cellMetrics->UsedPRBs = 100;
					// cellMetrics->TotDLAvailPRBs = 193;
					// cellMetrics->TotNumServUEs = 5;
					// cellMetrics->bitmap_servUEs = 0b0000000101010101;
					// strncpy((char *)(cellMetrics->CellID), "1113", sizeof("1113"));
					if (strcmp(gCellID, "1111") != 0) {
						printf("Cell ID is %s (This value should not be <1111>)\n", gCellID);
						if (flag_cell) {
							// 
							printf("TEST: CellMetric Size to send: %d\n", sizeof(CellMetricsEntry));
							mysend_socket((char *)cellMetrics, agent_ip, sizeof(CellMetricsEntry));
							printf("======== cellMetric sent over socket  =========\n");
							// DumpHex((char *)cellMetrics, sizeof(CellMetricsEntry));
							printf("CellMetric - MetricType: 0x%lX\n", cellMetrics->MetricType);
							printf("CellMetric - MeasUnixTime_msec: %lu\n", cellMetrics->MeasUnixTime_msec);
							printf("CellMetric - UsedPRBs: %ld\n", cellMetrics->UsedPRBs);
							printf("CellMetric - TotDLAvailPRBs: %ld\n", cellMetrics->TotDLAvailPRBs);
							printf("CellMetric - TotNumServUEs: %ld\n", cellMetrics->TotNumServUEs);
							printf("CellMetric - bitmap_servUEs: %x\n", cellMetrics->bitmap_servUEs);
							printf("CellMetric - CellID: %s\n", cellMetrics->CellID);
						}
					}
					flag_cell = false;
					free(cellMetrics);

					//test - UE Metrics
					// int test1 = sizeof(UeMetricsEntry); // 3 * sizeof(NeighborCellRFType) + sizeof(CellRFType) + 8 * 4 + 4 = 96 ==> __attribute__((__packed__))
					// int test2 = sizeof(CellRFType); //4 * 3 = 12
					// int test3 = sizeof(NeighborCellRFType); // 4 + sizeof(CellRFType) = 16
					// printf("test1 : %d, test2: %d, test3: %d\n", test1, test2, test3);
					break;
				}
				case 25:  // RIC indication header for E2SM-KPM
				{
					//struct tm *tm_info;
					//time_t unix_timestamp;

					int hdr_size = ricIndication->protocolIEs.list.array[idx]-> value.choice.RICindicationHeader.size;
					printf("hdr_size: %d\n", hdr_size);
					char* header = (char*) calloc(hdr_size, sizeof(char));
					memcpy(header, ricIndication->protocolIEs.list.array[idx]-> \
																		 value.choice.RICindicationHeader.buf, hdr_size);
					//printf("Header %s\n", header);
					printf("======== RIC indication header =========\n");
					DumpHex(header, hdr_size);

					// E2SM-KPM Details
					// 1. asn1c_defs Folder: add some header files and c files into asn1c_defs: 
					// 1-1) header files: E2SM-KPM-IndicationHeader.h, E2SM-KPM-IndicationHeader-Format1.h, TimeStamp.h
					// 1-2) c files: E2SM-KPM-IndicationHeader.c, E2SM-KPM-IndicationHeader-Format1.c, TimeStamp.c
					// 1-3) delete "asn_OER_.... " part
					// 2. e2sm folder: insert some part into e2sm_indication.hpp/c

					E2SM_KPM_IndicationHeader_t* decodedHdr = e2sm_decode_ric_indication_header(header, hdr_size);
					//printf("///////////E2sm////decodedHdr= %x\n",decodedHdr);

					unsigned long indHeader_present = (unsigned long) decodedHdr->present;
					//printf("present: %ld\n", indHeader_present);
					if (indHeader_present == 1) {
						printf("////entered indHeader_present = 1 \n");
						E2SM_KPM_IndicationHeader_Format1_t *indHeaderFormat1 = (decodedHdr->choice.indicationHeader_Format1);
						//printf("pass2 \n");
						unsigned long colTimeStamp_size = (unsigned long)(indHeaderFormat1->collectionStartTime.size);
						printf("colTimeStamp_size: %ld\n", colTimeStamp_size); // its isze is 8
						printf("--- collectionStartTime: byte string ---\n");
						DumpHex((char *)(indHeaderFormat1->collectionStartTime.buf), colTimeStamp_size);
						uint64_t valTimestamp =*(uint64_t *)(indHeaderFormat1->collectionStartTime.buf);
						valTimestamp = REVERSE_64LONG(valTimestamp);
						// printf("unix time (ms): %lu, unix time (sec): %lu\n", valTimestamp, (uint64_t)(floor(valTimestamp/1000.0)));
						// unix_timestamp = (time_t) (floor(valTimestamp/1000.0)); // msec --> sec
						// tm_info = localtime(&unix_timestamp); //gmtime : localtime
						// printf("Year: %d, Month: %d, Day: %d, Hour: %d, Minute: %d, Second: %d, msec: %lu\n",  tm_info->tm_year + 1900, \
						//  																			tm_info->tm_mon + 1, \
						// 																			tm_info->tm_mday, \
						// 																			tm_info->tm_hour + 9,\
						// 																			tm_info->tm_min,\
						// 																			tm_info->tm_sec, \
						// 																			(valTimestamp - (uint64_t)(floor(valTimestamp/1000.0)) * 1000));
						gMeasUnixTime_msec = valTimestamp;

						int KPMnodeIDType = (int) (indHeaderFormat1->id_GlobalE2node_ID.present);
						if (KPMnodeIDType == 1) {
							// To get Cell ID:
							char lCellID[8] = "0000000";
							printf("--- GlobalE2node_gNB_ID --- \n");
							printf("--- --- plmn_id --- --- \n");
							struct GlobalE2node_gNB_ID KPMnode = *(indHeaderFormat1->id_GlobalE2node_ID.choice.gNB);
							/*
							// error: variable 'GlobalE2node_gNB_ID KPMnode' has initializer but incomplete type
							// Resolution: add some header files in msgs_proc.hpp
							*/

							int gNB_plmn_id_size = (int) (KPMnode.global_gNB_ID.plmn_id.size);
							DumpHex((char *)(KPMnode.global_gNB_ID.plmn_id.buf), gNB_plmn_id_size);
							strncpy(lCellID, (char *)(KPMnode.global_gNB_ID.plmn_id.buf), gNB_plmn_id_size);
							//printf("test1 Cel ID: %s\n", lCellID);
							
							int gNB_gnb_id_size = (int) (KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.size);
							int gNB_gnb_id_BitsUnused = (int) (KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.bits_unused);
							printf("--- --- gnb_id (Unused bits: %d) --- --- \n", gNB_gnb_id_BitsUnused);
							DumpHex((char *)(KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.buf), gNB_gnb_id_size);
							strncpy(lCellID+3, (char *)(KPMnode.global_gNB_ID.gnb_id.choice.gnb_ID.buf), 1);
							lCellID[4] = '\0';
							//printf("test2 Cel ID: %s\n", lCellID);
							strncpy(gCellID, lCellID, strlen(lCellID));
							gCellID[4] = '\0';
							//printf("Cell ID: %s\n", gCellID);

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
							/*
							// error: variable 'GlobalE2node_eNB_ID KPMnode' has initializer but incomplete type
							*/
							int eNB_plmn_id_size = (int) (KPMnode.global_eNB_ID.pLMN_Identity.size);
							DumpHex((char *)(KPMnode.global_eNB_ID.pLMN_Identity.buf), eNB_plmn_id_size);
							strncpy(lCellID, (char *)(KPMnode.global_eNB_ID.pLMN_Identity.buf), eNB_plmn_id_size);

							int eNB_Type = (int) (KPMnode.global_eNB_ID.eNB_ID.present);
							if (eNB_Type == 1) {
								int macro_eNB_id_size = (int) (KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.size);
								int macro_eNB_id_BitsUnused = (int) (KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.bits_unused);
								printf("--- --- macro_eNB_ID (Unused bits: %d) --- --- \n", macro_eNB_id_BitsUnused);
								DumpHex((char *)(KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.buf), macro_eNB_id_size);
								strncpy(lCellID+3, (char *)(KPMnode.global_eNB_ID.eNB_ID.choice.macro_eNB_ID.buf), 1);
								lCellID[4] = '\0';
								//printf("test2 Cel ID: %s\n", lCellID);
								strncpy(gCellID, lCellID, strlen(lCellID));
								gCellID[4] = '\0';
								//printf("Cell ID: %s\n", gCellID);


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



