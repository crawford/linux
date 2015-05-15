/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QIOMMU_HW_H__
#define __QIOMMU_HW_H__

/*
 * The following structs act as "handles" to the QIOMMU config structures
 * defined in the QIOMMU specification.  Accessors for reading and writing
 * individual fields in the config structures are defined below.
 */
struct qiommu_ste {
	uint64_t raw[4];
} __aligned(0x20);

struct qiommu_ctx {
	uint64_t raw[8];
} __aligned(0x40);

struct qiommu_cmd {
	uint64_t raw[2];
} __aligned(0x10);

struct qiommu_evt {
	uint64_t raw[4];
} __aligned(0x20);

/*
 * The DEFINE_QIOMMU_{WR|RD}_FIELD_FN macros defined below are used to generate
 * accesor functions for fields in the QIOMMU HW config structures.
 *
 * The input parameters to the macro are:
 *   parent    : Portion of the structure name after qiommu_
 *   field     : Name of the field
 *   lsb       : Bit index in the parent structure where the field starts
 *   msb       : Bit index in the parent structure where the field ends
 *   val_shift : Number of bits that field values should be right shifted
 *               before being written or left shifted after being read.
 *               Typically only required for fields that contain pointers.
 *
 * The "output" of the WR macro will be a function named:
 *    void wr_<parent>__<field>(struct qiommu_<parent> *p, uint64_t val);
 *
 * The "output" of the RD macro will be a function named:
 *    uint64_t rd_<parent>__<field>(struct qiommu_<parent> *p);
 *
 * Prerequisites for correctness of the generated functions:
 *  - No field may be wider than 64 bits since that is the max supported width
 *    for the GENMASK_ULL macro.
 *  - Fields may not straddle a 64-bit boundary. ASSERT((lsb/64)==(msb/64)).
 *
 * Notes/Warnings:
 *  - Volatile pointers are not used in these macros so if the program order of
 *    field writes is important, explicit synchronization is required.
 */
#define DEFINE_QIOMMU_STRUCT_WR_FIELD_FN(parent, field, lsb, msb, val_shift)\
static inline \
void wr_##parent##__##field(struct qiommu_##parent *p, uint64_t val) \
{ \
	uint64_t *p64 = p->raw + (lsb) / 64; \
	uint64_t field_mask = GENMASK_ULL((msb) % 64, (lsb) % 64); \
	val = (val >> (val_shift)) << ((lsb) % 64); \
	*p64 = (*p64 & ~field_mask) | (val & field_mask); \
}

#define DEFINE_QIOMMU_STRUCT_RD_FIELD_FN(parent, field, lsb, msb, val_shift)\
static inline \
uint64_t rd_##parent##__##field(struct qiommu_##parent *p) \
{ \
	uint64_t *p64 = p->raw + (lsb) / 64; \
	uint64_t field_mask = GENMASK_ULL((msb) % 64, (lsb) % 64); \
	uint64_t field_val = (*p64 & field_mask) >> ((lsb) % 64); \
	return field_val << (val_shift); \
}

#define DEFINE_QIOMMU_STRUCT_ACCESSORS(parent, field, lsb, msb, io_shift) \
	DEFINE_QIOMMU_STRUCT_WR_FIELD_FN(parent, field, lsb, msb, io_shift) \
	DEFINE_QIOMMU_STRUCT_RD_FIELD_FN(parent, field, lsb, msb, io_shift)

/* Accessors for struct qiommu_ste */
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, V,                0,   0,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, Config,           1,   3,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, S1ContextPtr,     6,  47,   6)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, CONT,            56,  59,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, HD,              60,  60,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, HA,              61,  61,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, PIPA,            63,  63,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, DSS,             64,  65,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, CIR,             66,  67,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, COR,             68,  69,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, CSH,             70,  71,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, CDMax,           72,  76,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, S1Fmt,           77,  78,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, TTFmt,           79,  80,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, WXN,             82,  82,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, UWXN,            83,  83,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, S,               88,  88,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, R,               89,  89,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, A,               90,  90,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, STALLD,          91,  91,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, EATS,            92,  93,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, STRW,            94,  95,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, MemAttr,         96,  99,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, MTCFG,          100, 100,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ALLOCCFG,       101, 104,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, SHCFG,          108, 109,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, NSCFG,          110, 111,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, PRIVCFG,        112, 113,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, INSTCFG,        114, 115,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, S1_VMID,        128, 143,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ATS_MemAttr,    144, 147,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ATS_WA,         148, 148,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ATS_RA,         149, 149,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ATS_TR,         150, 150,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, ATS_SH,         151, 152,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ste, PRIPASID,       153, 153,   0)

/* Accessors for struct qiommu_ctx */
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, T0SZ,             0,   5,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, TG0,              6,   7,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, IR0,              8,   9,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, OR0,             10,  11,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, SH0,             12,  13,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, EPD0,            14,  14,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, ENDI,            15,  15,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, T1SZ,            16,  21,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, TG1,             22,  23,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, IR1,             24,  25,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, OR1,             26,  27,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, SH1,             28,  29,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, EPD1,            30,  30,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, V,               31,  31,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, IPS,             32,  34,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, AFFD,            35,  35,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, TBI,             38,  39,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, PAN,             40,  40,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, S,               44,  44,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, R,               45,  45,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, A,               46,  46,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, ASET,            47,  47,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, ASID,            48,  63,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, NSCFG0,          64,  64,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, HAD0,            65,  65,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, TTB0,            68, 111,   4)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, NSCFG1,         128, 128,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, HAD1,           129, 129,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, TTB1,           132, 175,   4)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, MAIR,           192, 255,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(ctx, AMAIR,          256, 319,   0)

/* Accessors for struct qiommu_cmd */
enum {
	CMD_INV_ALL   = 0x008,
	CMD_TLBI_ALL  = 0x010,
	CMD_TLBI_ASID = 0x011,
	CMD_TLBI_VA   = 0x012,
	CMD_SYNC      = 0x034,
};
DEFINE_QIOMMU_STRUCT_ACCESSORS(cmd, OPCODE,           0,   9,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(cmd, VMID,            32,  47,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(cmd, ASID,            48,  63,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(cmd, ADDR,            76, 127,  12)

/* Accessors for struct qiommu_evt */
enum QIOMMU_EVENT_TYPE {
	C_BAD_STREAMID         = 0x01,
	C_BAD_S_SSTI           = 0x02,
	F_UUT                  = 0x04,
	F_STE_FETCH            = 0x10,
	C_BAD_STE              = 0x11,
	F_VMTE_FETCH           = 0x20,
	C_BAD_VMTE             = 0x21,
	F_CD_FETCH             = 0x30,
	C_BAD_CD               = 0x31,
	C_BAD_SUBSTREAMID      = 0x33,
	F_BAD_AT_REQ           = 0x40,
	F_STREAM_DISABLED      = 0x42,
	F_TRANSL_FORBIDDEN     = 0x44,
	C_BAD_VMIDX            = 0x48,
	F_WALK_EABT            = 0x80,
	F_TRANSLATION          = 0x81,
	F_ADDRESS_SIZE         = 0x82,
	F_ACCESS               = 0x84,
	F_PERMISSION           = 0x88,
	F_TLB_CONFLICT         = 0xC0,
	E_CMDQ_ERR             = 0xC1,
	E_PRIQ_ERR             = 0xC2,
	E_MSI_ERR              = 0xC4,
	E_BAD_RESUME           = 0xE0,
	E_RCAT_SAFE_MODE_ERR   = 0xE1,
	IMPDEF_EVENT           = 0xFF
};

DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, Type,             0,   7,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, RnW,              8,   8,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, Priv,             9,   9,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, Inst,            10,  10,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, NS,              11,  11,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, Stage,           12,  12,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, SID,             32,  63,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, SSD,             64,  95,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, IPA_63_32,      128, 159,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, ASID,           160, 175,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, IPA_15_0,       160, 175,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, IPA_31_16,      176, 191,   0)
DEFINE_QIOMMU_STRUCT_ACCESSORS(evt, VA,             192, 255,   0)

#endif /* __QIOMMU_HW_H__ */
