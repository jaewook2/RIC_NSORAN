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
 * From ASN.1 module "E2AP-PDU-Contents"
 * 	found in "../asnTextFiles/e2ap-v01.00.00.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -fno-include-deps -findirect-choice -gen-PER -no-gen-OER -D.`
 */

#ifndef	_RANfunctionID_Item_H_
#define	_RANfunctionID_Item_H_


#include "asn_application.h"

/* Including external dependencies */
#include "RANfunctionID.h"
#include "RANfunctionRevision.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RANfunctionID-Item */
typedef struct RANfunctionID_Item {
	RANfunctionID_t	 ranFunctionID;
	RANfunctionRevision_t	 ranFunctionRevision;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RANfunctionID_Item_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RANfunctionID_Item;

#ifdef __cplusplus
}
#endif

#endif	/* _RANfunctionID_Item_H_ */
#include "asn_internal.h"
