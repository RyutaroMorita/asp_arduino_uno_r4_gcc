/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 * 
 *  ��L���쌠�҂́C�ȉ���(1)�`(4)�̏����𖞂����ꍇ�Ɍ���C�{�\�t�g�E�F
 *  �A�i�{�\�t�g�E�F�A�����ς������̂��܂ށD�ȉ������j���g�p�E�����E��
 *  �ρE�Ĕz�z�i�ȉ��C���p�ƌĂԁj���邱�Ƃ𖳏��ŋ�������D
 *  (1) �{�\�t�g�E�F�A���\�[�X�R�[�h�̌`�ŗ��p����ꍇ�ɂ́C��L�̒���
 *      ���\���C���̗��p��������щ��L�̖��ۏ؋K�肪�C���̂܂܂̌`�Ń\�[
 *      �X�R�[�h���Ɋ܂܂�Ă��邱�ƁD
 *  (2) �{�\�t�g�E�F�A���C���C�u�����`���ȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł���`�ōĔz�z����ꍇ�ɂ́C�Ĕz�z�ɔ����h�L�������g�i���p
 *      �҃}�j���A���Ȃǁj�ɁC��L�̒��쌠�\���C���̗��p��������щ��L
 *      �̖��ۏ؋K����f�ڂ��邱�ƁD
 *  (3) �{�\�t�g�E�F�A���C�@��ɑg�ݍ��ނȂǁC���̃\�t�g�E�F�A�J���Ɏg
 *      �p�ł��Ȃ��`�ōĔz�z����ꍇ�ɂ́C���̂����ꂩ�̏����𖞂�����
 *      �ƁD
 *    (a) �Ĕz�z�ɔ����h�L�������g�i���p�҃}�j���A���Ȃǁj�ɁC��L�̒�
 *        �쌠�\���C���̗��p��������щ��L�̖��ۏ؋K����f�ڂ��邱�ƁD
 *    (b) �Ĕz�z�̌`�Ԃ��C�ʂɒ�߂���@�ɂ���āCTOPPERS�v���W�F�N�g��
 *        �񍐂��邱�ƁD
 *  (4) �{�\�t�g�E�F�A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����邢���Ȃ鑹
 *      �Q������C��L���쌠�҂����TOPPERS�v���W�F�N�g��Ɛӂ��邱�ƁD
 *      �܂��C�{�\�t�g�E�F�A�̃��[�U�܂��̓G���h���[�U����̂����Ȃ闝
 *      �R�Ɋ�Â�����������C��L���쌠�҂����TOPPERS�v���W�F�N�g��
 *      �Ɛӂ��邱�ƁD
 * 
 *  �{�\�t�g�E�F�A�́C���ۏ؂Œ񋟂���Ă�����̂ł���D��L���쌠�҂�
 *  ���TOPPERS�v���W�F�N�g�́C�{�\�t�g�E�F�A�Ɋւ��āC����̎g�p�ړI
 *  �ɑ΂���K�������܂߂āC�����Ȃ�ۏ؂��s��Ȃ��D�܂��C�{�\�t�g�E�F
 *  �A�̗��p�ɂ�蒼�ړI�܂��͊ԐړI�ɐ����������Ȃ鑹�Q�Ɋւ��Ă��C��
 *  �̐ӔC�𕉂�Ȃ��D
 * 
 *  @(#) $Id$
 */


/*
 *  �R�A�ˑ��̓��ꖽ�߂̃C�����C���֐���`�iARM-M�p�j
 */

#ifndef CORE_INSN_H
#define CORE_INSN_H

#include <arm_m.h>

/*
 *  ���������ύX����邱�Ƃ��R���p�C���ɓ`���邽�߂̃}�N��
 */
#define ARM_MEMORY_CHANGED Asm("":::"memory")

/*
 *  FAULTMASK�̃Z�b�g
 */
Inline void
set_faultmask(void){
	Asm("cpsid f":::"memory");
}

/*
 *  FAULTMASK�̃N���A
 */
Inline void
clear_faultmask(void){
	Asm("cpsie f":::"memory");
}

/*
 *  PRIMASK�̃Z�b�g
 */
Inline void
set_primask(void){
	Asm("cpsid i":::"memory");
}

/*
 *  PRIMASK�̃N���A
 */
Inline void
clear_primask(void){
	Asm("cpsie i":::"memory");
}

/*
 *  PRIMASK�̃��[�h
 */
Inline uint32_t
read_primask(void){
	uint32_t val;

	Asm("mrs  %0, PRIMASK" : "=r"(val));
	return val;
}

/*
 *  BASEPRI�̃Z�b�g
 */
Inline void
set_basepri(uint32_t val){
	Asm("msr BASEPRI, %0" : : "r"(val) : "memory");
}

/*
 *  BASEPRI�̎擾
 */
Inline uint32_t
get_basepri(void){
	uint32_t val;
	Asm("mrs  %0, BASEPRI" : "=r"(val));
	return(val);
}

/*
 *  CONTROL�̃Z�b�g
 */
Inline void
set_control(uint32_t val){
	/*
	 *  control���W�X�^�Z�b�g��ɂ�isb���K�{
	 *  [ARMv7-M Architecture Reference Manaual(DDI0403B) A3-37]
	 */
	Asm("msr control, %0 \n"
		" isb"
		: : "r"(val) : "memory");
}

/*
 *  CONTROL�̎擾
 */
Inline uint32_t
get_control(void){
	uint32_t val;
	Asm("mrs  %0, CONTROL" : "=r"(val));
	return(val);
}

/*
 *  �X�e�[�^�X���W�X�^�iCPSR�j�̌��ݒl�̓Ǐo��
 */
Inline uint32_t
get_ipsr(void)
{
    uint32_t sr;
    Asm("mrs  %0, ipsr" : "=r"(sr));
    return(sr);
}

#endif /* CORE_INSN_H */
