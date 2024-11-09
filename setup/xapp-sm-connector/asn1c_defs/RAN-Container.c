/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "E2SM-KPM-RC"
 * 	found in "e2sm-kpm-rc.asn"
 * 	`asn1c -fcompound-names -fno-include-deps -findirect-choice -pdu=auto -gen-PER -gen-OER -no-gen-example -D .`
 */

#include "RAN-Container.h"

/*
 * This type is implemented using OCTET_STRING,
 * so here we adjust the DEF accordingly.
 */
static const ber_tlv_tag_t asn_DEF_RAN_Container_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (4 << 2))
};
asn_TYPE_descriptor_t asn_DEF_RAN_Container = {
	"RAN-Container",
	"RAN-Container",
	&asn_OP_OCTET_STRING,
	asn_DEF_RAN_Container_tags_1,
	sizeof(asn_DEF_RAN_Container_tags_1)
		/sizeof(asn_DEF_RAN_Container_tags_1[0]), /* 1 */
	asn_DEF_RAN_Container_tags_1,	/* Same as above */
	sizeof(asn_DEF_RAN_Container_tags_1)
		/sizeof(asn_DEF_RAN_Container_tags_1[0]), /* 1 */
	{ 0, 0, OCTET_STRING_constraint },
	0, 0,	/* No members */
	&asn_SPC_OCTET_STRING_specs	/* Additional specs */
};

