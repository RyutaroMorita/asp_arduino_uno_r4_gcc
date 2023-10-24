/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
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
 *  �^�C�}�h���C�o�iSYSTIC�p�j
 */
#include <sil.h>
#include "kernel_impl.h"
#include "time_event.h"
#include "target_timer.h"

/*
 *  �^�C�}�̋N������
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK    cyc;
	uint32_t tmp;

#ifdef SYSTIC_USE_CALIBRATION
	/* CALIBRATION���W�X�^�̒l���g�p */
	cyc = (sil_rew_mem((void *)SYSTIC_CALIBRATION) & SYSTIC_TENMS) / 10;
#else 
	cyc = TO_CLOCK(TIC_NUME, TIC_DENO) - 1;
#endif /* SYSTIC_USE_CALIBRATION */

	/* ��~ */
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp &= ~SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);

	sil_wrw_mem((void *)SYSTIC_RELOAD_VALUE, cyc);
	sil_wrw_mem((void *)SYSTIC_CURRENT_VALUE, cyc);

	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);

#ifdef SYSTIC_USE_STCLK
	/* �O���N���b�N�̎g�p */
	tmp |= SYSTIC_ENABLE;
#else
	/* �v���Z�b�T�N���b�N�̎g�p */
	tmp |= SYSTIC_ENABLE|SYSTIC_CLKSOURCE;
#endif /* SYSTIC_USE_STCLK */

	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
}

/*
 *  �^�C�}�̒�~����
 */
void
target_timer_terminate(intptr_t exinf)
{
	/* �^�C�}���~ */
	sil_wrw_mem((void*)SYSTIC_CONTROL_STATUS, 0x00);
}

/*
 *  �^�C�}�����݃n���h��
 */
void
target_timer_handler(void)
{
	/* probe_int �̂��߁CCOUNTFLAG���N���A */
	sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);

	i_begin_int(INTNO_TIMER);
	signal_time();                    /* �^�C���e�B�b�N�̋��� */
	i_end_int(INTNO_TIMER);
}
