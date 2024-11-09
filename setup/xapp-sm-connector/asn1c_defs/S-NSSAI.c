/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "E2SM-KPM-RC"
 * 	found in "e2sm-kpm-rc.asn"
 * 	`asn1c -fcompound-names -fno-include-deps -findirect-choice -pdu=auto -gen-PER -gen-OER -no-gen-example -D .`
 */

#include "S-NSSAI.h"

static int
memb_sST_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size == 1)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_sD_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	const OCTET_STRING_t *st = (const OCTET_STRING_t *)sptr;
	size_t size;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	size = st->size;
	
	if((size == 3)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

//static asn_oer_constraints_t asn_OER_memb_sST_constr_2 CC_NOTUSED = {
//	{ 0, 0 },
//	1	/* (SIZE(1..1)) */};
static asn_per_constraints_t asn_PER_memb_sST_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  1,  1 }	/* (SIZE(1..1)) */,
	0, 0	/* No PER value map */
};
//static asn_oer_constraints_t asn_OER_memb_sD_constr_3 CC_NOTUSED = {
//	{ 0, 0 },
//	3	/* (SIZE(3..3)) */};
static asn_per_constraints_t asn_PER_memb_sD_constr_3 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 0,  0,  3,  3 }	/* (SIZE(3..3)) */,
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_S_NSSAI_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct S_NSSAI, sST),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, &asn_PER_memb_sST_constr_2,  memb_sST_constraint_1 },
		0, 0, /* No default value */
		"sST"
		},
	{ ATF_POINTER, 1, offsetof(struct S_NSSAI, sD),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_OCTET_STRING,
		0,
		{ 0, &asn_PER_memb_sD_constr_3,  memb_sD_constraint_1 },
		0, 0, /* No default value */
		"sD"
		},
};
static const int asn_MAP_S_NSSAI_oms_1[] = { 1 };
static const ber_tlv_tag_t asn_DEF_S_NSSAI_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_S_NSSAI_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* sST */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* sD */
};
asn_SEQUENCE_specifics_t asn_SPC_S_NSSAI_specs_1 = {
	sizeof(struct S_NSSAI),
	offsetof(struct S_NSSAI, _asn_ctx),
	asn_MAP_S_NSSAI_tag2el_1,
	2,	/* Count of tags in the map */
	asn_MAP_S_NSSAI_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_S_NSSAI = {
	"S-NSSAI",
	"S-NSSAI",
	&asn_OP_SEQUENCE,
	asn_DEF_S_NSSAI_tags_1,
	sizeof(asn_DEF_S_NSSAI_tags_1)
		/sizeof(asn_DEF_S_NSSAI_tags_1[0]), /* 1 */
	asn_DEF_S_NSSAI_tags_1,	/* Same as above */
	sizeof(asn_DEF_S_NSSAI_tags_1)
		/sizeof(asn_DEF_S_NSSAI_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_S_NSSAI_1,
	2,	/* Elements count */
	&asn_SPC_S_NSSAI_specs_1	/* Additional specs */
};

