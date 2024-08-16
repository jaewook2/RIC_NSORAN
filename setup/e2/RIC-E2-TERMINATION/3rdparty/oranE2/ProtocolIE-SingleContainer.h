/*
 *
 * Copyright 2020 AT&T Intellectual Property
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "E2AP-Containers"
 * 	found in "../asnTextFiles/e2ap-v01.00.00.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -fno-include-deps -findirect-choice -gen-PER -no-gen-OER -D.`
 */

#ifndef	_ProtocolIE_SingleContainer_H_
#define	_ProtocolIE_SingleContainer_H_


#include "asn_application.h"

/* Including external dependencies */
#include "ProtocolIE-Field.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ProtocolIE-SingleContainer */
typedef RICaction_ToBeSetup_ItemIEs_t	 ProtocolIE_SingleContainer_1547P0_t;
typedef RICaction_Admitted_ItemIEs_t	 ProtocolIE_SingleContainer_1547P1_t;
typedef RICaction_NotAdmitted_ItemIEs_t	 ProtocolIE_SingleContainer_1547P2_t;
typedef RANfunction_ItemIEs_t	 ProtocolIE_SingleContainer_1547P3_t;
typedef RANfunctionID_ItemIEs_t	 ProtocolIE_SingleContainer_1547P4_t;
typedef RANfunctionIDcause_ItemIEs_t	 ProtocolIE_SingleContainer_1547P5_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P0;
asn_struct_free_f ProtocolIE_SingleContainer_1547P0_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P0_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P0_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P0_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P0_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P0_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P0_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P0_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P0_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P0_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P0_encode_aper;
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P1;
asn_struct_free_f ProtocolIE_SingleContainer_1547P1_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P1_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P1_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P1_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P1_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P1_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P1_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P1_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P1_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P1_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P1_encode_aper;
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P2;
asn_struct_free_f ProtocolIE_SingleContainer_1547P2_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P2_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P2_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P2_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P2_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P2_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P2_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P2_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P2_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P2_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P2_encode_aper;
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P3;
asn_struct_free_f ProtocolIE_SingleContainer_1547P3_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P3_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P3_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P3_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P3_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P3_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P3_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P3_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P3_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P3_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P3_encode_aper;
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P4;
asn_struct_free_f ProtocolIE_SingleContainer_1547P4_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P4_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P4_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P4_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P4_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P4_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P4_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P4_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P4_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P4_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P4_encode_aper;
extern asn_TYPE_descriptor_t asn_DEF_ProtocolIE_SingleContainer_1547P5;
asn_struct_free_f ProtocolIE_SingleContainer_1547P5_free;
asn_struct_print_f ProtocolIE_SingleContainer_1547P5_print;
asn_constr_check_f ProtocolIE_SingleContainer_1547P5_constraint;
ber_type_decoder_f ProtocolIE_SingleContainer_1547P5_decode_ber;
der_type_encoder_f ProtocolIE_SingleContainer_1547P5_encode_der;
xer_type_decoder_f ProtocolIE_SingleContainer_1547P5_decode_xer;
xer_type_encoder_f ProtocolIE_SingleContainer_1547P5_encode_xer;
per_type_decoder_f ProtocolIE_SingleContainer_1547P5_decode_uper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P5_encode_uper;
per_type_decoder_f ProtocolIE_SingleContainer_1547P5_decode_aper;
per_type_encoder_f ProtocolIE_SingleContainer_1547P5_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ProtocolIE_SingleContainer_H_ */
#include "asn_internal.h"
