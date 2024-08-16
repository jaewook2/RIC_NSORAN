/*
 * Copyright 2019 AT&T Intellectual Property
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
 */

/*
 * This source code is part of the near-RT RIC (RAN Intelligent Controller)
 * platform project (RICP).
 */



/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "X2AP-Containers"
 * 	found in "../../asnFiles/X2AP-Containers.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -fno-include-deps -findirect-choice -gen-PER -no-gen-OER -D.`
 */

#ifndef	_PrivateIE_Field_H_
#define	_PrivateIE_Field_H_


#include "asn_application.h"

/* Including external dependencies */
#include "PrivateIE-ID.h"
#include "Criticality.h"
#include "ANY.h"
#include "asn_ioc.h"
#include "OPEN_TYPE.h"
#include "constr_CHOICE.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PrivateMessage_IEs__value_PR {
	PrivateMessage_IEs__value_PR_NOTHING	/* No components present */
	
} PrivateMessage_IEs__value_PR;

/* PrivateIE-Field */
typedef struct PrivateMessage_IEs {
	PrivateIE_ID_t	 id;
	Criticality_t	 criticality;
	struct PrivateMessage_IEs__value {
		PrivateMessage_IEs__value_PR present;
		union PrivateMessage_IEs__value_u {
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} value;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PrivateMessage_IEs_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PrivateMessage_IEs;
extern asn_SEQUENCE_specifics_t asn_SPC_PrivateMessage_IEs_specs_1;
extern asn_TYPE_member_t asn_MBR_PrivateMessage_IEs_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _PrivateIE_Field_H_ */
#include "asn_internal.h"
