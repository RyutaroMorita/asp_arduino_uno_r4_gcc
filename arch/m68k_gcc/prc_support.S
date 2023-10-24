/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2010 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: prc_support.S 2728 2015-12-30 01:46:11Z ertl-honda $
 */

/*
 *		�v���Z�b�T�ˑ����W���[�� �A�Z���u�����ꕔ�iM68040�p�j
 */

#define	TOPPERS_MACRO_ONLY
#define UINT_C(val)			(val)		/* uint_t�^�̒萔�����}�N�� */
#define ULONG_C(val)		(val)		/* ulong_t�^�̒萔�����}�N�� */
#define CAST(type, val)		(val)		/* �^�L���X�g���s���}�N�� */
#include "kernel_impl.h"
#include "offset.h"

/*
 *  �^�X�N�f�B�X�p�b�`��
 */
	.text
	.globl dispatch
dispatch:
	/*
	 *  ���̃��[�`���́C�^�X�N�R���e�L�X�g�ECPU���b�N��ԁE�f�B�X�p�b�`
	 *  ����ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂŌĂяo��
	 *  ���D
	 */
	movem.l %d2-%d7/%a2-%a6, -(%sp)		/* ���W�X�^��ۑ� */
	move.l p_runtsk, %a0				/* p_runtsk��A0�� */
	move.l %sp, TCB_msp(%a0)			/* �^�X�N�X�^�b�N��ۑ� */
	move.l #dispatch_r, TCB_pc(%a0)		/* ���s�ĊJ�Ԓn��ۑ� */
	jbra dispatcher

dispatch_r:
	movem.l (%sp)+, %d2-%d7/%a2-%a6		/* ���W�X�^�𕜋A */
	btst.b #TCB_enatex_bit, TCB_enatex(%a0)
	jbeq dispatch_r_1					/* enatex��false�Ȃ烊�^�[�� */
	tst.l TCB_texptn(%a0)				/* texptn��0�Ȃ烊�^�[�� */
	jbeq dispatch_r_1
	tst.l ipmflg						/* ipmflg��true�ł����            */
	jbne call_texrtn					/*  �^�X�N��O�������[�`���̌ďo�� */
dispatch_r_1:
	rts

/*
 *  �f�B�X�p�b�`���̓���J�n�iprc_support.S�j
 */
	.globl start_dispatch
start_dispatch:
	/*
	 *  ���̃��[�`���́C�J�[�l���N�����ɁC���ׂĂ̊����݂��֎~�������
	 * �i�����݃��b�N��ԂƓ����j�ŌĂяo�����D�܂��C�����݃��[�h�i��
	 *  �^�X�N�R���e�L�X�g�Ɠ����j�ŌĂяo����邱�Ƃ�z�肵�Ă���D
	 *
	 *  prc_initialize�ŁClock_flag��true�ɁCsaved_iipm��IIPM_ENAALL��
	 *  ���������Ă��邽�߁C�J�[�l���Ǘ��O�̊����݂������邱�ƂŁC
	 *  CPU���b�N��ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂɂȂ�D
	 *  �܂��Cinitialize_task��disdsp��false�ɏ��������Ă��邽�߁C�f�B
	 *  �X�p�b�`����ԂɂȂ��Ă���D
	 */
#if TIPM_LOCK == -7
	or.w #0x1000, %sr					/* �}�X�^���[�h�� */
#else /* TIPM_LOCK == -7 */
	move.w %sr, %d0						/* �}�X�^���[�h�E                */
	and.w #~0x0700, %d0					/*  �J�[�l���Ǘ��O�̊����݂����� */
	or.w #(0x1000 | IIPM_LOCK), %d0
	move.w %d0, %sr
#endif /* TIPM_LOCK == -7 */
	jbra dispatcher_0

/*
 *  ���݂̃R���e�L�X�g���̂Ăăf�B�X�p�b�`
 */
	.globl exit_and_dispatch
exit_and_dispatch:
	/* �f�B�X�p�b�`���{�́idispatcher�j�� */

/*
 *  �f�B�X�p�b�`���{��
 */
dispatcher:
	/*
	 *  ���̃��[�`���́C�^�X�N�R���e�L�X�g�ECPU���b�N��ԁE�f�B�X�p�b�`
	 *  ����ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂŌĂяo��
	 *  ���D
	 *
	 *  ���Ȃ킿�C�}�X�^���[�h�Elock_flag��true�Edisdsp��false�Edspflg
	 *  ��true�Esaved_iipm��IIPM_ENAALL�ƂȂ��Ă���D���s�ĊJ�Ԓn�ւ���
	 *  �̏�Ԃ̂܂ܕ��򂷂�D
	 */
#ifdef LOG_DSP_ENTER
	move.l p_runtsk, %d0				/* p_runtsk���p�����[�^�� */
	move.l %d0, -(%sp)
	jsr log_dsp_enter
	addq.l #4, %sp
#endif /* LOG_DSP_ENTER */
dispatcher_0:
	move.l p_schedtsk, %a0				/* p_schedtsk��p_runtsk�� */
	move.l %a0, p_runtsk
	jbeq dispatcher_1					/* p_runtsk��NULL�Ȃ�dispatcher_1�� */
	move.l TCB_msp(%a0), %sp			/* �^�X�N�X�^�b�N�𕜋A */
#ifdef LOG_DSP_LEAVE
	move.l %a0, -(%sp)					/* p_runtsk���p�����[�^�� */
	jsr log_dsp_leave
	addq.l #4, %sp
	move.l p_runtsk, %a0
#endif /* LOG_DSP_LEAVE */
	move.l TCB_pc(%a0), %a1				/* ���s�ĊJ�Ԓn�֕��� */
	jmp (%a1)
dispatcher_1:
	/*
	 *  CPU���b�N��Ԃ��������鏀��������D
	 */
	move.w %sr, %d0						/* SR��D0�ɕۑ� */
	clr.l lock_flag						/* CPU���b�N������Ԃ� */
dispatcher_2:
	/*
	 *  �����݂������C�����݃��[�h�ɐ؂芷���āC�����݂�҂D
	 *
	 *  �����Ŋ����݃��[�h�ɐ؂芷����̂́C�����Ŕ������銄���ݏ�����
	 *  �ǂ̃X�^�b�N���g�����Ƃ������̉����ƁC�����݃n���h�����ł̃^
	 *  �X�N�f�B�X�p�b�`�̖h�~�Ƃ���2�̈Ӗ�������D
	 *
	 *  �v���Z�b�T�������ݑ҂��Ɉڍs�����鏈���ƁC�����݋��Ƃ́C�s��
	 *  ���ɍs�Ȃ��K�v������iM68040�ł�stop���߂ŗ����s�Ȃ��̂Ŗ���
	 *  ���j�D�����s���ɍs�Ȃ�Ȃ��ꍇ�C�����݂�����������Ɋ���
	 *  �݂�����C���̒��Ń^�X�N�����s�\��ԂɂȂ�ƁC���s���ׂ��^�X
	 *  �N������ɂ�������炸�v���Z�b�T�������ݑ҂��ɂȂ��Ă��܂��D
	 *
	 *  �����ݑ҂��̊Ԃ́Cp_runtsk��NULL�i��0�j�ɐݒ肵�Ȃ���΂Ȃ��
	 *  ���D���̂悤�ɐݒ肵�Ȃ��ƁC�����݃n���h������iget_tid���Ăяo
	 *  �����ۂ̓��삪�d�l�ɍ��v���Ȃ��Ȃ�D
	 */
	stop #0x2000						/* �����ݑ҂� */
	move.w %d0, %sr						/* ���̏�Ԃɖ߂� */
	tst.l reqflg						/* reqflg��false�Ȃ�dispatcher_2�� */
	jbeq dispatcher_2
	clr.l reqflg						/* reqflg��false�� */
	/*
	 *  CPU���b�N��Ԃɖ߂��D�����ݑ҂��̊ԂɎ��s���������݃n���h���ɂ�
	 *  ��Csaved_iipm��������������\�������邽�߁C���̒l�ɖ߂��K
	 *  �v������Ddispatcher�����s����鎞�́Csaved_iipm��IIPM_ENAALL
	 *  �ƂȂ��Ă��邽�߁C�����ł�saved_iipm��IIPM_ENAALL�i��0�j�ɖ߂�
	 *  �΂悢�D
	 */
	clr.w saved_iipm					/* saved_iipm��0�ɂ��� */
	move.l #true, lock_flag				/* CPU���b�N��Ԃ� */
	jbra dispatcher_0

/*
 *  �J�[�l���̏I�������̌ďo��
 *
 *  M68040�ł́C���[�h�؊����ɂ��X�^�b�N���؂芷��邽�߁C�����I�ȃX
 *  �^�b�N�؊����͕K�v�Ȃ��D�ŏ����犄���݃��[�h�ł������ꍇ���C������
 *  ���ł悢�D
 */
	.globl call_exit_kernel
call_exit_kernel:
	and.w #~0x1000, %sr					/* �����݃��[�h�� */
	jmp exit_kernel						/* �J�[�l���̏I���������Ă� */
	
/*
 *  �^�X�N�J�n������
 */
	.text
	.globl start_r
start_r:
	clr.l lock_flag						/* CPU���b�N������Ԃ� */
	and.w #~0x0700, %sr
	move.l TCB_p_tinib(%a0), %a1		/* p_runtsk->p_tinib��A1�� */
	move.l TINIB_exinf(%a1), -(%sp)		/* exinf���X�^�b�N�ɐς� */
	move.l #ext_tsk, -(%sp)				/* ext_tsk���X�^�b�N�ɐς� */
	move.l TINIB_task(%a1), %a0			/* �^�X�N�̋N���Ԓn��A0�� */
	jmp (%a0)

/*
 *  �����݃n���h���o������
 *
 *  ret_int�́C�����݃n���h������߂�������Ɏ��s���郋�[�`���ŁC
 *  INTHDR_ENTRY�}�N���œW�J���銄���݃n���h���̓��������ŁC�����݃n��
 *  �h������̖߂�Ԓn�ɐݒ肳���D
 */
	.text
	.globl ret_int
ret_int:
	btst.b #4, 16(%sp)					/* �߂�悪�����݃��[�h�Ȃ� */
	jbeq ret_int_1						/*           �����Ƀ��^�[�� */
	/*
	 *  �J�[�l���Ǘ��̊����݂��֎~����D���̎��_�ł́CCPU���b�N��Ԃ�
	 *  �͂Ȃ�Ȃ��ilock_flag��saved_iipm�͍X�V���Ȃ��j�D
	 *
	 *  reqflg���`�F�b�N����O�Ɋ����݂��֎~����̂́Creqflg���`�F�b�N
	 *  ��������Ɋ����݃n���h�����N������C���̒��Ńf�B�X�p�b�`���v��
	 *  ���ꂽ�ꍇ�ɁC�����Ƀf�B�X�p�b�`����Ȃ��Ƃ�����肪�����邽��
	 *  �ł���D
	 */
#if TIPM_LOCK == -7
	or.w #0x0700, %sr					/* ���ׂĂ̊����݂��֎~ */
#else /* TIPM_LOCK == -7 */
	/*
	 *  ���̃��[�`���̓J�[�l���Ǘ��̊����݂���N������邽�߁C�����ł́C
	 *  �����ݗD��x�}�X�N��TIPM_LOCK�����Ⴂ�Ƒz��ł���D
	 */
	move.w %sr, %d0						/* �J�[�l���Ǘ��̊����݂��֎~ */
	and.w #~0x0700, %d0
	or.w #IIPM_LOCK, %d0
	move.w %d0, %sr
#endif /* TIPM_LOCK == -7 */
	tst.l reqflg						/* reqflg��true�ł����ret_int_2�� */
	jbne ret_int_2
ret_int_1:
	/*
	 *  �����ݏ�������̃��^�[���ɂ��CCPU���b�N������ԂɈڍs����悤
	 *  ��������D�����ݗD��x�}�X�N�́CRTE���߂ɂ�茳�̒l�ɖ߂邽�߁C
	 *  lock_flag��false�ɂ��Ă����΂悢�ilock_flag��false�ɂ���̂́C
	 *  CPU���b�N��Ԃ̂܂܊����݃n���h������߂������̑΍�j�D
	 */
	clr.l lock_flag						/* CPU���b�N�����̏��� */
	movem.l (%sp)+, %d0-%d1/%a0-%a1		/* �X�N���b�`���W�X�^�𕜋A */
	rte

ret_int_2:
	movem.l (%sp)+, %d0-%d1/%a0-%a1		/* �X�N���b�`���W�X�^�𕜋A */
	addq.l #8, %sp						/* �X���[�A�E�F�C�t���[�����̂Ă� */
	or.w #0x1000, %sr					/* �}�X�^���[�h�Ɂi�X�^�b�N�؊����j*/
	movem.l %d0-%d1/%a0-%a1, -(%sp)		/* �X�N���b�`���W�X�^��ۑ� */
ret_int_3:
	/*
	 *  �����ւ́CCPU��O�n���h���̏o��������������򂵂Ă���D
	 *
	 *  �����ł́C�߂�悪�^�X�N�ł���C�X�^�b�N�́C��O�X�^�b�N�t���[
	 *  ���̏�ɃX�N���b�`���W�X�^�݂̂��ۑ����ꂽ��ԂɂȂ��Ă���D��
	 *  ���C�v���Z�b�T�́C�}�X�^���[�h�E�J�[�l���Ǘ��̊����݂��֎~����
	 *  ��ԂƂȂ��Ă���D
	 */
	clr.l reqflg						/* reqflg��false�� */
	/*
	 *  CPU���b�N��ԂɈڍs���C�����ݗD��x�}�X�N�������ݏ����O�̒l�ɐ�
	 *	�肷��D�J�[�l���Ǘ��̊����݂͂��łɋ֎~���Ă���̂ŁClock_flag
	 *  ��saved_iipm���X�V����Dsaved_iipm�́C�߂��̊����ݗD��x�}�X
	 *  �N�i�̓����\���j�ɐݒ肷��D
	 *
	 *  ���̎��_��CPU���b�N��ԂƂ���̂́Cdispatcher�֕��򂷂鎞�ƁC
	 *  call_texrtn���Ăяo�����ɁCCPU���b�N��ԂɂȂ��Ă���K�v������
	 *  ���߂ł���D
	 */
	move.w 16(%sp), %d0					/* �߂���SR��D0�� */
	and.w #0x0700, %d0					/* IPM�����o����saved_iipm�� */
	move.w %d0, saved_iipm
	move.l #true, lock_flag				/* lock_flag��true�� */
	/*
	 *  dspflg��false�ł���ꍇ�ƁCp_runtsk��p_schedtsk�������ꍇ�ɂ́C
	 *  �f�B�X�p�b�`���s��Ȃ��D���̃`�F�b�N���K�v�Ȃ̂́C�^�X�N��O��
	 *  �����[�`���̌ďo�����K�v�ȏꍇ�ɁC�f�B�X�p�b�`���K�v�Ȃ��Ă��C
	 *  reqflg��true�ɂ��邽�߂ł���D
	 */
	move.l p_runtsk, %a0				/* p_runtsk��A0�� */
	tst.l dspflg						/* dspflg��false�Ȃ�ret_int_4�� */
	jbeq ret_int_4
	cmp.l p_schedtsk, %a0				/* p_runtsk��p_schedtsk�������Ȃ� */
	jbeq ret_int_4						/*                    ret_int_4�� */
	movem.l %d2-%d7/%a2-%a6, -(%sp)		/* �c��̃��W�X�^��ۑ� */
	move.l %sp, TCB_msp(%a0)			/* �^�X�N�X�^�b�N��ۑ� */
	move.l #ret_int_r, TCB_pc(%a0)		/* ���s�ĊJ�Ԓn��ۑ� */
	jbra dispatcher

ret_int_r:
	movem.l (%sp)+, %d2-%d7/%a2-%a6		/* ���W�X�^�𕜋A */
ret_int_4:
	/*
	 *  enatex��true�ŁCtexptn��0�łȂ��Cipmflg��true�ł���΁C�^�X�N
	 *  ��O�������[�`�����Ăяo���D
	 */
	btst.b #TCB_enatex_bit, TCB_enatex(%a0)
	jbeq ret_int_5						/* enatex��false�Ȃ�ret_int_5�� */
	tst.l TCB_texptn(%a0)				/* texptn��0�Ȃ�ret_int_5�� */
	jbeq ret_int_5
	tst.l ipmflg						/* ipmflg��false�Ȃ�ret_int_5�� */
	jbeq ret_int_5
	jsr call_texrtn						/* �^�X�N��O�������[�`���̌ďo�� */
ret_int_5:
	/*
	 *  �����ݏ�������̃��^�[���ɂ��CCPU���b�N������ԂɈڍs�����
	 *  ����������D�����ݗD��x�}�X�N�́CRTE���߂ɂ�茳�̒l�ɖ߂邽
	 *  �߁Clock_flag��false�ɂ��Ă����΂悢�D
	 */
	clr.l lock_flag						/* CPU���b�N�����̏��� */
	movem.l (%sp)+, %d0-%d1/%a0-%a1		/* �X�N���b�`���W�X�^�𕜋A */
	rte

/*
 *  CPU��O�n���h���o��������
 *
 *  exchdr_entry�́CCPU��O�������������Ɏ��s���郋�[�`���ŁCEXCHDR_ENTRY
 *  �}�N���œW�J����CPU��O�n���h���̓�����������CCPU��O�n���h���̔Ԓn��
 *  A1�ɁCCPU��O�n���h���ԍ���D1�ɓ���ĕ��򂵂Ă���D
 *
 *  CPU��O�n���h���́C��^�X�N�R���e�L�X�g�Ŏ��s����D���̂��߁CCPU��
 *  �O�n���h�����Ăяo���O�Ɋ����݃��[�h�Ɉڍs���C���^�[�����Ă������
 *  ���̃��[�h�ɖ߂��D���̃��[�h�ɖ߂����߂ɁC�����݃��[�h�Ɉڍs����O
 *  ��SR�������݃X�^�b�N��ɕۑ�����i���^�[�����SR���Q�Ƃ���������
 *  ���C�^�X�N�X�^�b�N��ɕۑ������ꍇ������C�Q�Ƃ���̂��ʓ|�j�D
 */
	.text
	.globl exchdr_entry
exchdr_entry:
	lea.l 16(%sp), %a0					/* ��O�t���[���̐擪��A0�� */
	move.w %sr, %d0						/* SR��D0�� */
	and.w #~0x1000, %sr					/* �����݃��[�h�Ɂi�X�^�b�N�؊����j*/
	move.l %d0, -(%sp)					/* ����SR���X�^�b�N�ɕۑ� */
	and.w #0x0700, %d0					/* IPM��INT_IPM(TMIN_INTPRI) */
	cmp.w #INT_IPM(TMIN_INTPRI), %d0	/*    ��菬������΁C�J�[�l�� */
	blt exchdr_entry_1					/*    �Ǘ���CPU��O�̏����� */

	/*
	 *  �J�[�l���Ǘ��O��CPU��O�n���h���̌ďo��
	 */
	move.l lock_flag, %d0				/* ����lock_flag���X�^�b�N�ɕۑ� */
	move.l %d0, -(%sp)
	move.l %a0, -(%sp)					/* A0���p�����[�^�� */
	jsr (%a1)							/* CPU��O�n���h�����Ăяo�� */
	addq.l #4, %sp						/* �X�^�b�N��̈������̂Ă� */
	move.l (%sp)+, %d0					/* lock_flag�����ɖ߂� */
	move.l %d0, lock_flag
	move.l (%sp)+, %d0					/* SR�����ɖ߂� */
	move.w %d0, %sr
	movem.l (%sp)+, %d0-%d1/%a0-%a1		/* �X�N���b�`���W�X�^�𕜋A */
	rte

	/*
	 *  �J�[�l���Ǘ���CPU��O�n���h���̌ďo��
	 */
exchdr_entry_1:
#ifdef LOG_EXC_LEAVE					/* excno_num��log_exc_leave�� */
	move.l %d1, -(%sp)					/*               �p�����[�^�� */
#endif /* LOG_EXC_LEAVE */
	move.l %a0, -(%sp)					/* A0��CPU��O�n���h���̃p�����[�^�� */
#ifdef LOG_EXC_ENTER
	move.l %a1, -(%sp)					/* A1��ۑ� */
	move.l %d1, -(%sp)					/* excno_num���p�����[�^�� */
	jsr _kernel_log_exc_enter			/* log_exc_enter���Ăяo�� */
	addq.l #4, %sp
	move.l (%sp)+, %a1					/* A1�𕜋A */
#endif /* LOG_EXC_ENTER */
	jsr (%a1)							/* CPU��O�n���h�����Ăяo�� */
	addq.l #4, %sp						/* �X�^�b�N��̈������̂Ă� */
#ifdef LOG_EXC_LEAVE
	jsr _kernel_log_exc_leave			/* log_exc_leave���Ăяo�� */
	addq.l #4, %sp
#endif /* LOG_EXC_LEAVE */
ret_exc:
	move.l (%sp)+, %d0					/* ����SR��D0�� */
	and.w #0x1000, %d0					/* ���������݃��[�h�Ȃ� */
	jbeq ret_exc_1						/*       �����Ƀ��^�[�� */
	/*
	 *  �J�[�l���Ǘ��̊����݂��֎~���C�}�X�^���[�h�ɖ߂��D���̎��_�ł́C
	 *  CPU���b�N��Ԃɂ͂Ȃ�Ȃ��ilock_flag��saved_iipm�͍X�V���Ȃ��j�D
	 *
	 *  reqflg���`�F�b�N����O�Ɋ����݂��֎~����̂́Creqflg���`�F�b�N
	 *  ��������Ɋ����݃n���h�����N������C���̒��Ńf�B�X�p�b�`���v��
	 *  ���ꂽ�ꍇ�ɁC�����Ƀf�B�X�p�b�`����Ȃ��Ƃ�����肪�����邽��
	 *  �ł���D
	 */
#if TIPM_LOCK == -7
	or.w #0x1700, %sr					/* �}�X�^���[�h�E�S�����݋֎~ */
#else /* TIPM_LOCK == -7 */
	/*
	 *  ���̃��[�`���̓J�[�l���Ǘ���CPU��O����N������邽�߁C�����ł́C
	 *  �����ݗD��x�}�X�N��TIPM_LOCK�����Ⴂ�Ƒz��ł���D
	 */
	move.w %sr, %d0						/* �}�X�^���[�h�E                */
	and.w #~0x0700, %d0					/*    �J�[�l���Ǘ��̊����݂��֎~ */
	or.w #(0x1000|IIPM_LOCK), %d0
	move.w %d0, %sr
#endif /* TIPM_LOCK == -7 */
	/*
	 *  CPU��O�ł͊����݃��[�h�ɐ؂芷���Ȃ����߁Creqflg��true�̏ꍇ�C
	 *  ret_int_2�ł͂Ȃ��Cret_int_3�֕��򂷂�D
	 */
	tst.l reqflg						/* reqflg��true�ł����ret_int_3�� */
	jbne ret_int_3
ret_exc_1:
	movem.l (%sp)+, %d0-%d1/%a0-%a1		/* �X�N���b�`���W�X�^�𕜋A */
	rte

/*
 *  �������ԑ҂��i�{����SIL�̃^�[�Q�b�g�ˑ����j
 */
	.globl _sil_dly_nse
_sil_dly_nse:
	subi.l #SIL_DLY_TIM1, %d0			/* D0����SIL_DLY_TIM1������ */
	jbhi _sil_dly_nse_1					/* ���ʂ�0�ȉ��Ȃ烊�^�[�� */
	rts
_sil_dly_nse_1:
	subi.l #SIL_DLY_TIM2, %d0			/* D0����SIL_DLY_TIM2������ */
	jbhi _sil_dly_nse_1					/* ���ʂ�0���傫����΃��[�v */
	rts