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
 * From ASN.1 module "X2AP-IEs"
 * 	found in "../../asnFiles/X2AP-IEs.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -fno-include-deps -findirect-choice -gen-PER -no-gen-OER -D.`
 */

#include "UL-InterferenceOverloadIndication.h"

asn_per_constraints_t asn_PER_type_UL_InterferenceOverloadIndication_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 7,  7,  1,  110 }	/* (SIZE(1..110)) */,
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_UL_InterferenceOverloadIndication_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (10 << 2)),
		0,
		&asn_DEF_UL_InterferenceOverloadIndication_Item,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_UL_InterferenceOverloadIndication_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
asn_SET_OF_specifics_t asn_SPC_UL_InterferenceOverloadIndication_specs_1 = {
	sizeof(struct UL_InterferenceOverloadIndication),
	offsetof(struct UL_InterferenceOverloadIndication, _asn_ctx),
	1,	/* XER encoding is XMLValueList */
};
asn_TYPE_descriptor_t asn_DEF_UL_InterferenceOverloadIndication = {
	"UL-InterferenceOverloadIndication",
	"UL-InterferenceOverloadIndication",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_UL_InterferenceOverloadIndication_tags_1,
	sizeof(asn_DEF_UL_InterferenceOverloadIndication_tags_1)
		/sizeof(asn_DEF_UL_InterferenceOverloadIndication_tags_1[0]), /* 1 */
	asn_DEF_UL_InterferenceOverloadIndication_tags_1,	/* Same as above */
	sizeof(asn_DEF_UL_InterferenceOverloadIndication_tags_1)
		/sizeof(asn_DEF_UL_InterferenceOverloadIndication_tags_1[0]), /* 1 */
	{ 0, &asn_PER_type_UL_InterferenceOverloadIndication_constr_1, SEQUENCE_OF_constraint },
	asn_MBR_UL_InterferenceOverloadIndication_1,
	1,	/* Single element */
	&asn_SPC_UL_InterferenceOverloadIndication_specs_1	/* Additional specs */
};

