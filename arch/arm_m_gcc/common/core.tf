$ 
$     �p�X2�̃A�[�L�e�N�`���ˑ��e���v���[�g�iARM-M�p�j
$ 

$ 
$  �L���Ȋ����ݔԍ��C�����݃n���h���ԍ�
$ 
$INTNO_VALID = RANGE(15, TMAX_INTNO)$
$INHNO_VALID = INTNO_VALID$

$ 
$  �L����CPU��O�ԍ�
$ 
$EXCNO_VALID = { 2,3,4,5,6,11,12,14 }$

$ 
$  ATT_ISR�Ŏg�p�ł��銄���ݔԍ��Ƃ���ɑΉ����銄���݃n���h���ԍ�
$ 
$INTNO_ATTISR_VALID = INTNO_VALID$
$INHNO_ATTISR_VALID = INHNO_VALID$

$ 
$  DEF_INT�^DEF_EXC�Ŏg�p�ł��銄���݃n���h���ԍ��^CPU��O�n���h���ԍ�
$ 
$INHNO_DEFINH_VALID = INHNO_VALID$
$EXCNO_DEFEXC_VALID = EXCNO_VALID$

$ 
$  CFG_INT�Ŏg�p�ł��銄���ݔԍ��Ɗ����ݗD��x
$  �ő�D��x��BASEPRI���W�X�^�Ń}�X�N�ł��Ȃ��D��x�i�����D��x'0'�j
$  ���̂��߁C�J�[�l���Ǘ��O�̊����݂ł̂ݎw��\�D
$INTNO_CFGINT_VALID = INTNO_VALID$
$INTPRI_CFGINT_VALID = RANGE(-(1 << TBITW_IPRI),-1)$
           
$ 
$  �W���e���v���[�g�t�@�C���̃C���N���[�h
$ 
$INCLUDE "kernel/kernel.tf"$

/*$NL$
$SPC$*  Target-dependent Definitions (ARM-M)$NL$
$SPC$*/$NL$
$NL$

$ 
$  �x�N�^�[�e�[�u��
$ 
$FILE "kernel_cfg.c"$
$NL$

$IF ISFUNCTION("GEN_VECTOR_TABLE_VARNAME")$
	$GEN_VECTOR_TABLE_VARNAME()$
$ELSE$
	$IF ISFUNCTION("VECTOR_ATTRIBUTE")$
		$VECTOR_ATTRIBUTE()$
	$ELSE$
		__attribute__ ((section(".vector"))) $NL$
	$END$
	const FP _kernel_vector_table[] =      $NL$ 
$END$
{                                    $NL$
	$TAB$(FP)(TOPPERS_ISTKPT(TOPPERS_ISTK, TOPPERS_ISTKSZ)), /* 0 The initial stack pointer */$NL$
	$TAB$(FP)_kernel__start,                    /* 1 The reset handler */$NL$

$FOREACH excno {2,3,...,14}$ 
	$IF (excno == 11) && (__TARGET_ARCH_THUMB == 4) $
		$TAB$(FP)(_kernel_svc_handler),        /* 11 SVCall handler */$NL$
	$ELSE$
		$IF (excno == 14) && (__TARGET_ARCH_THUMB == 3) $
			$TAB$(FP)(pendsvc_handler),        /* 14 PandSVCall handler */$NL$
		$ELSE$
			$TAB$(FP)(_kernel_core_exc_entry),$SPC$$FORMAT("/* %d */", +excno)$$NL$
		$END$
	$END$
$END$

$FOREACH inhno INTNO_VALID$ 
	$IF LENGTH(INH.INHNO[inhno]) && ((INH.INHATR[inhno] & TA_NONKERNEL) != 0)$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_core_int_entry),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$


$NL$};$NL$
$NL$

$NL$
const FP _kernel_exc_tbl[] = $NL$
{$NL$
$FOREACH excno {0,1,...,14}$
	$IF LENGTH(EXC.EXCNO[excno])$
		$TAB$(FP)($EXC.EXCHDR[excno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_exc_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +excno)$$NL$
$END$


$FOREACH inhno INTNO_VALID$
	$IF LENGTH(INH.INHNO[inhno])$
		$TAB$(FP)($INH.INTHDR[inhno]$),
	$ELSE$
		$TAB$(FP)(_kernel_default_int_handler),
	$END$
	$SPC$$FORMAT("/* %d */", +inhno)$$NL$
$END$


$NL$};$NL$
$NL$

$ 
$  _kernel_bitpat_cfgint�̐���
$ 

$bitpat_cfgint_num = 0$
$bitpat_cfgint = 0$


const uint32_t _kernel_bitpat_cfgint[
$IF (TMAX_INTNO & 0x0f) == 0x00 $
	$bitpat_cfgint_num = (TMAX_INTNO >> 4)$
$ELSE$
	$bitpat_cfgint_num = (TMAX_INTNO >> 4) + 1$
$END$
	$bitpat_cfgint_num$
] = {$NL$
$FOREACH num RANGE(0,(bitpat_cfgint_num-1))$
$   //boost �̃o�[�W�����ɂ���ċ������ς�邽�߂̑΍�
$   //http://www.toppers.jp/TOPPERS-USERS/201004/msg00034.html
	$bitpat_cfgint = 1-1$
	$FOREACH inhno RANGE(num*32, (num*32)+31)$
		$IF LENGTH(INH.INHNO[inhno])$
			$bitpat_cfgint = bitpat_cfgint | (1 << (inhno & 0x01f))$
		$END$
	$END$
	$TAB$UINT32_C($FORMAT("0x%08x", bitpat_cfgint)$), $NL$
$END$

$NL$};$NL$
$NL$




$IF __TARGET_ARCH_THUMB == 4 $

$ 
$  �����ݗD��x�e�[�u���i�����\���j
$ 
const uint32_t _kernel_int_iipm_tbl[] = {$NL$
$FOREACH excno {0,1,...,14}$
	$TAB$$FORMAT("UINT32_C(0x%08x), /* 0x%03x */", 0, +excno)$$NL$
$END$

$FOREACH intno INTNO_VALID$
	$IF LENGTH(INT.INTNO[intno])$
		$intpri = (((1 << TBITW_IPRI) + INT.INTPRI[intno]) << (8 - TBITW_IPRI))$
	$ELSE$
$		// LSB��1�ɂ��Ă���̂́C�����ݑ������ݒ肳��Ă��Ȃ����Ƃ�
$		// �ʂ��邽�߂ł���D
		$intpri = 0 $
	$END$
	$TAB$$FORMAT("UINT32_C(0x%08x), /* 0x%03x */", intpri, +intno)$$NL$
$END$
$NL$};$NL$
$NL$

$END$

$IF __TARGET_ARCH_THUMB == 3 $

$ 
$  �����ݗD��x�e�[�u���i�����\���j
$ 
const uint8_t _kernel_int_iipm_tbl[] = {$NL$
$FOREACH excno {0,1,...,14}$
	$TAB$$FORMAT("UINT8_C(0x%02x), /* 0x%03x */", 0, +excno)$$NL$
$END$

$FOREACH intno INTNO_VALID$
	$IF LENGTH(INT.INTNO[intno])$
		$intpri = (((1 << TBITW_IPRI) + INT.INTPRI[intno]))$
	$ELSE$
		$intpri = +255 $
	$END$
	$TAB$$FORMAT("UINT8_C(0x%02x), /* 0x%03x */", intpri, +intno)$$NL$
$END$
};$NL$
$NL$

$ 
$  �����ݗD��x�}�X�N���̊����݂������銄����
$ 
const uint32_t _kernel_iipm_enable_irq_tbl[]={$NL$
$FOREACH intpri {-4,-3,...,0}$
	$enable_mask = 0$
	$FOREACH intno RANGE(16, TMAX_INTNO)$
		$IF LENGTH(INT.INTNO[intno]) && (INT.INTPRI[intno] < intpri)$
			$enable_mask = enable_mask | (1 << (intno - 16))$
		$END$
	$END$
	$TAB$$FORMAT("UINT32_C(0x%08x), /* %d(%d) */", enable_mask, intpri+4, intpri)$$NL$
$END$
};$NL$
$NL$

const uint8_t _kernel_iipm_enable_systic_tbl[]={$NL$
$FOREACH intpri {-4,-3,...,0}$
	$enable_mask = 0$
	$IF LENGTH(INT.INTNO[15]) && (INT.INTPRI[15] < intpri)$
		$enable_mask = 1$
	$END$
	$TAB$$FORMAT("UINT8_C(0x%02x), /* %d(%d) */", enable_mask, intpri+4, intpri)$$NL$
$END$
};$NL$
$END$
