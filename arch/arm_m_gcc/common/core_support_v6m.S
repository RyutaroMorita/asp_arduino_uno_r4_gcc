/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2015 by Embedded and Real-Time Systems Laboratory
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
 *  �v���Z�b�T�ˑ����W���[�� �A�Z���u�����ꕔ�iARMv6-M�p�j
 */

#define TOPPERS_MACRO_ONLY
#define UINT_C(val)		(val)		/* uint_t�^�̒萔�����}�N�� */
#define ULONG_C(val)	(val)		/* ulong_t�^�̒萔�����}�N�� */
#define CAST(type, val)	(val)		/* �^�L���X�g���s���}�N�� */

#include "kernel_impl.h"
#include "arm_m.h"
#include "offset.h"
#include "target_asm.inc"

#if __TARGET_ARCH_THUMB != 3
#error The core_support.S support ARMv6-M only.
#endif /* __TARGET_ARCH_THUMB != 3 */

/*
 *  �^�X�N�f�B�X�p�b�`��
 */
	ATEXT
	ABALIGN(4)
	ATHUMB(dispatch)
	AGLOBAL(dispatch)
ALABEL(dispatch)
	/*
	 *
	 *  ���̃��[�`���́C�^�X�N�R���e�L�X�g�ECPU���b�N��ԁE�f�B�p�b�`�����
	 *  �E�i���f����́j�����ݗD��x�}�X�N�S�J��ԂŌĂяo�����D
	 */
	mov  r0, r8                  /* ���W�X�^�̕ۑ� */
	mov  r1, r9
	mov  r2, r10
	mov  r3, r11
	push {r0-r7,lr}
	ldr  r0, =p_runtsk           /* p_runtsk��ǂݍ��� */
	ldr  r1, [r0]
	mov  r2, sp
	str  r2, [r1,#TCB_sp]        /* �^�X�N�X�^�b�N��ۑ� */
	ldr  r3, =dispatch_r         /* ���s�ĊJ�Ԓn��ۑ� */
	str  r3, [r1,#TCB_pc]        
	b    dispatcher

	ATHUMB(dispatch_r)
	AGLOBAL(dispatch_r)
ALABEL(dispatch_r)
	pop  {r0-r7}                 /* ���W�X�^�̕��A */
	mov  r8,  r0
	mov  r9,  r1
	mov  r10, r2
	mov  r11, r3
	/*
	 * �^�X�N��O�������[�`���̋N��
	 */
	ldr  r0, =p_runtsk           /* p_runtsk��ǂݍ��� */
	ldr  r1, [r0]
	ldrb r0, [r1,#TCB_enatex]
	movs r2, #TCB_enatex_mask
	tst  r0, r2
	beq  dispatch_r_1            /* enatex �� false �Ȃ烊�^�[�� */
	ldr  r0, [r1,#TCB_texptn]    /* texptn �� 0 �Ȃ烊�^�[��     */
	tst  r0, r0
	beq  dispatch_r_1            
	ldr  r1, =ipmflg             /* ipmflg�� false �Ȃ烊�^�[��  */
	ldr  r0, [r1]
	tst  r0, r0
	beq  dispatch_r_1
	ldr  r0, =call_texrtn        /* �^�X�N��O���[�`���̌Ăяo�� */
	bx   r0
ALABEL(dispatch_r_1)             /* �^�X�N�ւ�call_textn����߂� */
	pop  {pc}

/*
 *  CPU��O�G���g��
 *
 *  �����݃G���g���Ə����̓��e�͓��������C���O�̎�ނ��قȂ邽�߁C
 *  �����Ă���D
 */
	ABALIGN(4)
	ATEXT
	ATHUMB(core_exc_entry)
	AGLOBAL(core_exc_entry)
ALABEL(core_exc_entry)
	/*
	 *  ��O/�����݂���������ƁC�������ɃA�N�e�B�u�ȃX�^�b�N�ɃX�N��
	 *  �b�`���W�X�^�����ۑ������D
	 *  ���̓��e�ɉ����āCCPU��O�n���h���ւ̏��Ƃ��āCbasepri �̒l�ƁC
	 *  EXC_RETURN�̏��������ĕۑ�����Dbasepri�̒l�́CCPU��O�����
	 *  ���^�[�����Ɋ����ݗD��x�}�X�N�̒l�����ɖ߂����߂ɂ��p������D
	 *
	 *   -----------
	 *  |   iipm    |
	 *   -----------
	 *  | EXC_RETURN|
	 *   -----------
	 *  |    R0     |  
	 *   -----------
	 *  |    R1     |
	 *   -----------
	 *  |    R2     |
	 *   -----------
	 *  |    R3     |
	 *   -----------
	 *  |    R12    |
	 *   -----------
	 *  |    LR     |
	 *   -----------
	 *  |    PC     |
	 *   -----------
	 *  |   xPSR    |
	 *   -----------
	 *
	 */

	/*
	 *  �J�[�l���Ǘ��O�̗�O���`�F�b�N
	 *  �J�[�l�����̃N���e�B�J���Z�N�V�����̎��s���C�S�����݃��b�N��ԁC
	 *  CPU���b�N��ԁC�J�[�l���Ǘ��O�̊����݃n���h�����s���̂����ꂩ��
	 *  ��������CPU��O���C�J�[�l���Ǘ��O��CPU��O�ƌĂ�
	 *  �S�����݃��b�N��Ԃ�FAULTMASK��'1'�̏ꍇ
	 *  CPU���b�N��Ԃ�basepri��IIPM_LOCK���Ŕ��f����D
	 */
	ldr   r3, =iipm               /* ��Ŏg�p���邽�ߓǂݍ��� */
	ldrb  r0, [r3]
	mrs   r2, primask             /* CPU���b�N��ԂȂ�J�[�l���Ǘ��O��O������ */
	cmp   r1, #1
	beq   core_nonkernel_exc_entry

	/*
	 * �X�^�b�N��ύX����K�v�����邩�`�F�b�N
	 * EXC_RETURN�i�����ݎ���LR�ɐݒ肳���l�j���`�F�b�N���āC��O��������
	 * �A�N�e�B�u�ȃX�^�b�N����肷�邱�Ƃő��d�����݂����肷��D
	 */
	movs  r1, #EXC_RETURN_PSP    /* �����݌���MSP�Ȃ瑽�d������ */
	mov   r2, lr
	tst   r2, r1
	beq   core_exc_entry_1       /* ���d�����݂Ȃ�core_exc_entry_1�� */
	mrs   r3, psp                /* ��i�ڂ̊����݂̏ꍇ��PSP��� */
	subs  r3, #(0x04*2)          /* �����ݔ������̊����ݗD��x�}�X�N(r2)�CEXC_RETURN(r0)�̏���PSP��ɐς� */
	str   r2, [r3, 0x04]
	str   r0, [r3, 0x00]
	msr   psp, r3
	push  {r0, r2}               /* MSP�ɂ������ݔ������̊����ݗD��x�}�X�N(r0)��EXC_RETURN(r2)��ς� */
	mov   lr, r3                 /* ����`�̊����݂����������ꍇ�̏��Ƃ��� */
	b     core_exc_entry_2
ALABEL(core_exc_entry_1)         /* ���d�����݂̏ꍇ */
	push  {r0,r2}                /* �����ݔ������̊����ݗD��x�}�X�N(r0)�CEXC_RETURN(r2)�̏���PSP��ɐς� */
	mov   lr, sp                 /* CPU��O�n���h���ւ̈����ƂȂ� */

	/*
	 *  ���ʏ���
	 */
ALABEL(core_exc_entry_2)
	/*
	 *  CPU��O���烊�^�[������ۂɊ����ݗD��x�}�X�N��߂����߂̃f�[�^��ۑ�
	 */
	ldr  r3, =SYSTIC_CONTROL_STATUS
	ldr  r2,[r3] 
	ldr  r3, =NVIC_SETENA0
	ldr  r1,[r3] 
	push {r1,r2}

	mrs   r3, ipsr               /* �n���h���A�h���X���擾 */
	ldr   r1, =_kernel_exc_tbl
	lsls  r0, r3, #2
	ldr   r2, [r1, r0]

#ifdef LOG_EXC_ENTER
	push  {r0,r2,r3}
	mov   r0, r3                 /* ��O�ԍ����p�����[�^��  */
	bl    log_exc_enter          /* log_exc_enter���Ăяo�� */
	pop   {r0,r2,r3}
	push  {r3}                   /* ��O�ԍ����X�^�b�N��    */
#endif /* LOG_EXC_ENTER */

	/*
	 *  CPU��O�n���h���̌Ăяo��
	 */
	mov   r0, lr
	blx   r2

#ifdef LOG_EXC_ENTER
	pop   {r0}                   /* ��O�ԍ���������        */
	bl    log_exc_leave          /* log_exc_leave���Ăяo�� */
#endif /* LOG_EXC_ENTER */

	b     ret_exc

/*
 *  �J�[�l���Ǘ��O��CPU��O�̏o��������
 */
ALABEL(core_nonkernel_exc_entry)
	movs  r1, #EXC_RETURN_PSP    /* �����݌���MSP�Ȃ瑽�d������ */
	mov   r2, lr
	tst   r2, r1
	beq   core_nonkernel_exc_entry_1  /* ���d�����݂Ȃ� */
	mrs   r3, psp                /* ��i�ڂ̊����݂̏ꍇ��PSP��� */
	subs  r3, #(0x04*2)
	str   r2, [r3, 0x04]
	str   r0, [r3, 0x00]
	msr   psp, r3                
	push  {r0, r2}               /* MSP�ɂ������ݔ������̊����ݗD��x�}�X�N(r0)��EXC_RETURN(r2)��ς� */
	mov   r0,  r3                /* CPU��O�n���h���ւ̈����ƂȂ� */
	b     core_nonkernel_exc_entry_2
ALABEL(core_nonkernel_exc_entry_1) /* ���d�����݂̏ꍇ */
	push  {r0, r2}               /* �����ݔ������̊����ݗD��x�}�X�N(r0)��EXC_RETURN(r2)��ς� */
	mov   r0, sp                 /* CPU��O�n���h���ւ̈����ƂȂ� */

	/*
	 *  ���ʏ���
	 */
ALABEL(core_nonkernel_exc_entry_2)
	/*
	 *  �����݂��烊�^�[������ۂɊ����ݗD��x�}�X�N��߂����߂̃f�[�^��ۑ�
	 */
	ldr  r3, =SYSTIC_CONTROL_STATUS
	ldr  r2,[r3] 
	ldr  r3, =NVIC_SETENA0
	ldr  r1,[r3] 
	push {r1,r2}

	mrs   r3, ipsr               /* CPU��O�n���h���̃A�h���X���擾 */
	ldr   r1, =_kernel_exc_tbl
	lsls  r4, r3, #2
	ldr   r2, [r1, r4]

	/*
	 *  CPU��O�n���h���̌Ăяo��
	 */
	blx   r2

	/*
	 *  CPU���b�N��Ԃ̏ꍇ�ɌĂяo����Ă��邽�߁CCPU��O�n���h������̃��^�[��
	 *  �̂��߂Ɍ��ɖ߂�
	 */
	cpsid i

	/*
	 *  �����ݗD��x�}�X�N�����ɖ߂�
	 */
	ldr  r1, =0xffffffff
	ldr  r2, =NVIC_CLRENA0  /* ��UIRQ���荞�݂�S�ċ֎~ */
	str  r1, [r2]
	pop  {r0,r1}            /* �X�^�b�N�����O��������NVIC_SETENA0(r0),SYSTIC_CONTROL_STATUS(r1)�̒l���擾 */
	ldr  r2, =NVIC_SETENA0  /* NVIC_SETENA0�����ɖ߂� */
	str  r0, [r2]
	ldr  r3, =SYSTIC_CONTROL_STATUS  /* SYSTIC_CONTROL_STATUS�����ɖ߂� */
	ldr  r0,[r3] 
	movs r2, #SYSTIC_TICINT
	orrs r1, r1, r2
	bics r0, r2
	orrs r0, r1
	str  r0, [r3]
	pop  {r0}               /* iipm��߂� */
	ldr  r1, =iipm
	strb r0, [r1]

	/*
	 *  �߂��̃R���e�L�X�g�̔���
	 * 
	 *  �����݃n���h�����s��LR�ɃZ�b�g�����EXC_RETURN���`�F�b�N���āC�߂�
	 *  ���MSP���g���Ă���΁C�����ݐ悪��^�X�N�R���e�L�X�g�Ɣ��肷��D
	 */
	pop   {r3}                   /* lr���X�^�b�N����擾        */
	movs  r1, #EXC_RETURN_PSP    /* �����݌���MSP�Ȃ瑽�d������ */
	tst   r3, r1
	bne   core_nonkernel_ret_exc_1
	b     core_nonkernel_ret_exc_2 /* �̒l��MSP����擾 */
ALABEL(core_nonkernel_ret_exc_1)
	/*
	 *  PSP�ォ��EXC_RETURN/���̊����ݗD��x�}�X�N(iipm)���폜
	 */
	mrs   r2, psp
	adds  r2, r2, #8
	msr   psp, r2

ALABEL(core_nonkernel_ret_exc_2)
	bx    r3                      /* ���^�[�� */

/*
 *  �����݃G���g��
 */
	ATHUMB(core_int_entry)
	AGLOBAL(core_int_entry)
ALABEL(core_int_entry)
	/*
	 *  �����ݔ������̊����ݗD��x�}�X�N���X�^�b�N�ɕۑ����邽�ߎ擾
	 */
	ldr   r3, =iipm               /* ��Ŏg�p���邽�ߓǂݍ��� */
	ldrb  r0, [r3]

	/*
	 * ���d�����݂��`�F�b�N
	 * EXC_RETURN�i�����ݎ���LR�ɐݒ肳���l�j���`�F�b�N���āC��O��������
	 * �A�N�e�B�u�ȃX�^�b�N����肷�邱�Ƃő��d�����݂����肷��D
	 */
	movs  r1, #EXC_RETURN_PSP    /* �����݌���MSP�Ȃ瑽�d������ */
	mov   r2, lr
	tst   r2, r1
	beq   core_int_entry_1       /* ���d�����݂Ȃ�core_int_entry_1�� */
	mrs   r3, psp                /* ��i�ڂ̊����݂̏ꍇ��PSP��� */
	subs  r3, #(0x04*2)
	str   r2, [r3, 0x04]
	str   r0, [r3, 0x00]
	msr   psp, r3
	push  {r0, r2}               /* MSP�ɂ������ݔ������̊����ݗD��x�}�X�N(r0)��EXC_RETURN(r2)��ς� */
	mov   lr, r3                 /* ����`�̊����݂����������ꍇ�̏��Ƃ��� */
	b     core_int_entry_2
ALABEL(core_int_entry_1)         /* ���d�����݂̏ꍇ */
	push  {r0, r2}               /* �����ݔ������̊����ݗD��x�}�X�N(r0)��EXC_RETURN(r2)��ς� */
	mov   lr, sp                 /* ����`�̊����݂����������ꍇ�̏��Ƃ��� */

	/*
	 *  ���ʏ���
	 */
ALABEL(core_int_entry_2)
	/*
	 *  �����݂��烊�^�[������ۂɊ����ݗD��x�}�X�N��߂����߂̃f�[�^��ۑ�
	 */
	ldr  r3, =SYSTIC_CONTROL_STATUS
	ldr  r2,[r3] 
	ldr  r3, =NVIC_SETENA0
	ldr  r1,[r3] 
	push {r1,r2}

	mrs   r3, ipsr   /* �����ݗv���ԍ��̎擾 */

	/*
	 *  basepri�̐ݒ�
	 *  NVIC�D��x�}�X�N�������I�ɐݒ肳��邽�ߗD��x�}�X�N�̓_�ł͕K�v��
	 *  �����Cx_get_ipm()��basepri���Q�Ƃ��邽�߁Cbasepri���X�V����D
	 */
	ldr   r1, =_kernel_int_iipm_tbl
	ldrb  r2, [r1, r3]
	ldr   r0, =iipm
	strb  r2, [r0]

	ldr   r1, =_kernel_exc_tbl   /* �n���h���A�h���X���擾 */
	lsls  r0, r3, #2
	ldr   r2, [r1, r0]

#ifdef LOG_INH_ENTER
	push  {r0,r2,r3}
	mov   r0, r3                 /* ��O�ԍ����p�����[�^��  */
	bl    log_inh_enter          /* log_exc_enter���Ăяo�� */
	pop   {r0,r2,r3}
#endif /* LOG_EXC_ENTER */

#ifdef LOG_INH_LEAVE
	mrs   r3, ipsr               /* �����ݗv���ԍ��̎擾 */
	push  { r3 }                 /* �����ݔԍ����X�^�b�N��    */
#endif /* LOG_INT_LEAVE */

	/*
	 *  �����݃n���h���̌Ăяo��
	 */
	mov   r0, lr
	blx   r2

#ifdef LOG_INH_LEAVE
	pop   {r0}                   /* ��O�ԍ���������        */
	bl    log_exc_leave          /* log_exc_leave���Ăяo�� */
#endif /* LOG_INH_LEAVE */


/*
 *  ������/��O�o��
 *
 *  ret_exc/ret_int�́CCPU��O/�����݃n���h������߂�������Ɏ��s����
 *  ���[�`���ł���D
 */
ALABEL(ret_exc)
ALABEL(ret_int)
	/*
	 *  �����݃��b�N��ԂƂ���D���̎��_�ł́CCPU���b�N��Ԃɂ͂Ȃ�Ȃ�
	 * �ibasepri��lock_flag��saved_iipm�͍X�V���Ȃ��j�D
	 *
	 *  �����݃��b�N��ԂƂ���̂́C�߂��̃R���e�L�X�g�̃`�F�b�N�ƁC
	 *  �߂�悪��^�X�N�R���e�L�X�g�ł������ꍇ�̃��^�[�����A�g�~�b�N
	 *  �ɍs�����߂ł���Dbsepri��CPU���b�N�̒l�ɂ��邱�Ƃł��A�g�~�b
	 *  �N�ȃ`�F�b�N�ƕ��A�͉\�ł��邪�C�����݂��烊�^�[�����Ă��C
	 *  basepri �̐ݒ���e�͌��ɖ߂�Ȃ����߁C�g�p���邱�Ƃ��ł��Ȃ��D 
	 *  ����CFAULTMASK�́C�����݂���̃��^�[�������ɂ���āC'0'�ɃN��
	 *  �A�����D
	 */
	cpsid i

	/*
	 *  �����ݗD��x�}�X�N�����ɖ߂�
	 */
	ldr  r1, =0xffffffff
	ldr  r2, =NVIC_CLRENA0  /* ��UIRQ���荞�݂�S�ċ֎~ */
	str  r1, [r2]
	pop  {r0,r1}            /* �X�^�b�N�����O/�����ݔ�������NVIC_SETENA0(r0),SYSTIC_CONTROL_STATUS(r1)�̒l���擾 */
	ldr  r2, =NVIC_SETENA0  /* NVIC_SETENA0�����ɖ߂� */
	str  r0, [r2]
	ldr  r3, =SYSTIC_CONTROL_STATUS  /* SYSTIC_CONTROL_STATUS�����ɖ߂� */
	ldr  r0,[r3] 
	movs r2, #SYSTIC_TICINT
	orrs r1, r1, r2
	bics r0, r2
	orrs r0, r1
	str  r0, [r3]
	pop  {r0}               /* iipm��߂� */
	ldr  r1, =iipm
	strb r0, [r1]

	/*
	 *  �߂��̃R���e�L�X�g�̔���
	 * 
	 *  �����݃n���h�����s��LR�ɃZ�b�g�����EXC_RETURN���`�F�b�N���āC�߂�
	 *  ���MSP���g���Ă���΁C�����ݐ悪��^�X�N�R���e�L�X�g�Ɣ��肷��D
	 */
	pop   {r3}                     /* lr���X�^�b�N����擾         */
	movs  r1, #EXC_RETURN_PSP      /* �߂�悪PSP�Ȃ� ret_int_1 �� */
	tst   r3, r1
	bne   ret_int_1
	b     ret_int_2

	/*
	 *  ��i�ڂ̊����݂̏o������
	 */
ALABEL(ret_int_1)
	/*
	 *  PSP�ォ��CEXC_RETURN(r0)�ƌ��̊����ݗD��x�}�X�N(basepri)(r1)
	 *  ���擾
	 */
	mrs   r2, psp
	adds  r2, r2, #(0x04*2)  
	msr   psp, r2

	/*
	 *  reqflg���`�F�b�N����
	 * 
	 *  �J�[�l���Ǘ����̊����݂͋֎~������ԂŎ��s����K�v�����邽�߁C
	 *  FAULTMASK��'1'�ɂ�����ԂŎ��s����D
	 *  reqflg���`�F�b�N����O�Ɋ����݂��֎~����̂́Creqflg���`�F�b�N
	 *  ��������Ɋ����݃n���h�����N������C���̒��Ńf�B�X�p�b�`���v��
	 *  ���ꂽ�ꍇ�ɁC�����Ƀf�B�X�p�b�`����Ȃ��Ƃ�����肪�����邽��
	 *  �ł���D
	 */
	ldr   r0, =reqflg             /* reqflg��false�Ȃ炻�̂܂ܖ߂� */
	ldr   r2, [r0]
	cmp   r2, #1                  /* true�Ȃ�ret_int_3��           */
	beq   ret_int_3

ALABEL(ret_int_2)
	/*
	 *  �����ɂ͊����݃��b�N��ԁiFAULTMASK���Z�b�g�j���ꂽ��Ԃŗ���D
	 *  Thread���[�h����̃��^�[���ɂ�莩���I�Ɋ����݃��b�N������ԂɂȂ�D
	 *  �����ݗD��x�}�X�N�͊����ݑO�ɏ�Ԃɖ߂��D
	 */ 
	cpsie i                       /* �����݂̃��^�[���ŃN���A����Ȃ����߃N���A */
	bx    r3                      /* ���^�[�� */

ALABEL(ret_int_3)
	/*
	 *  �����ł́C�߂�悪�^�X�N�ł���CPSP��ɃX�N���b�`���W�X�^�Ɗ�
	 *  ���ݗD��x�}�X�N(basepri)���ۑ����ꂽ��ԂɂȂ��Ă���D�܂��C
	 *  �v���Z�b�T�́CHandler���[�h�E�����݃��b�N��ԂƂȂ��Ă���D
	 *  �܂��Cr0�ɂ́Creqflg�̃A�h���X�Cr3�ɂ͊����ݎ�t����lr�̒l����
	 *  ������Ă���D
	 */
	/*
	 *  �^�X�N��O�n���h����f�B�X�p�b�`������ۂ�Thread���[�h�֑J�ڂ���
	 *  �_�~�[�̃X�^�b�N�t���[�����쐬���āCbx���߂�Handler���[�h���烊
	 *  �^�[������D�܂��C�x���f�B�X�p�b�`����ꍇ���C�Ăъ��荞�񂾃^�X
	 *  �N�ɖ߂�ۂɂ́Csvc���߂ŁCsvc_handler���Ăяo���D
	 *  �X�^�b�N�t���[���́CConfigureation and Control Register(CCR)��
	 *  STKALIGN��'1'�̏ꍇ�́C8byte���E�ɃA���C�������D
	 *  �Q�l : DDI0403B_arm_architecture_v7m_reference_manual(P.220)
	 *  ���̂��߁C���̎��_�̃X�^�b�N�͊����݂��O�������ɍ쐬���ꂽ
	 *  �X�^�b�N�t���[������C8byte���E�̃T�C�Y�ɂ��Ă����ƁCsvc_handler
	 *  ���ŃX�^�b�N�t���[���̃A���C�����g�̗L���̊m�F���ȗ��ł���D
	 *  �������C�V�X�e���N����́C���I��CCR��STKALIGN�̐ݒ��ύX����̂�
	 *  �֎~�Ƃ���D
	 *  ���̎��_�͕W���̃X�^�b�N�t���[���́C�����݁E��O�������Ɠ����ł�
	 *  �邽�߁C�^�X�N�X�^�b�N(PSP)��8byte���E�ɂȂ��Ă���D
	 */
	movs  r1, #0                   /* reqflg��false�� */
	str   r1, [r0]

	/*
	 *  CPU���b�N��ԂɈڍs����D
	 *
	 *  �J�[�l���̊Ǘ����̊����݂��֎~����悤��basepri��ݒ肵�C
	 *  lock_flag �� saved_iipm ���X�V����Dsaved_iipm�́C�߂��̊�����
	 *  �D��x�}�X�N�i�̓����\���j�ɐݒ肷��D
	 *  ���̎��_��CPU���b�N��ԂƂ���̂́Cdispatcher�֕��򂷂鎞�ƁC
	 *  call_texrtn���Ăяo�����ɁCCPU���b�N��ԂɂȂ��Ă���K�v������
	 *  ���߂ł���D
	 *  �Ȃ��C���̏����̌�CThread���[�h�ւ̈ڍs�������s�Ȃ����߁C������
	 *  ���b�N���(FAULTMASK��"1")�͕ێ�����D
	 */

	/*
	 *  �����ݗD��x�}�X�N���C�S������ԁiTIPM_ENAALL�j�ɐݒ肷��
	 *  ���ł�CPU���b�N��ԂȂ̂ŁCsaved_iipm��IIPM_ENAALL�Ƃ���D
	 */

	/*
	 *  Thread���[�h�ֈڍs����D
	 *
	 *  dispatcher��call_texrn���Ăяo���ꍇ�́CThread���[�h�ł���K
	 *  �v�����邽�߁CPSP�X�^�b�N��Ƀ_�~�[�̗�O�t���[����u���āC
	 *  �[���I�Ɋ����݃n���h�����烊�^�[������D
	 *  ���^�[���Ɠ�����FAULTMASK�������I�ɃN���A����C�J�[�l���Ǘ��O��
	 *  �����݂��������D
	 */
	ldr   r0, =ret_int_4          /* PC   */
	ldr   r1, =EPSR_T             /* xPSR(T�r�b�g��'1'�ł���K�v������) */
	mrs   r2, psp
	subs  r2, #(0x04*2)
	str   r1, [r2, 0x04]
	str   r0, [r2, 0x00]
	subs  r2, #(EXC_FRAME_SIZE - (4*2)) /* r0-r3,r12,lr�̓��e�͐ݒ肷��K�v���Ȃ� */
	msr   psp,r2                  
	bx    r3                      /* Thread���[�h�ֈڍs */
    
ALABEL(ret_int_4)
	/*
	 *  ��L�̏����ɂ��CThread���[�h�Ŏ��s�����D
	 *  dspflg��false�ł���ꍇ�ƁCp_runtsk��p_schedtsk�������ꍇ�ɂ́C
	 *  �f�B�X�p�b�`���s��Ȃ��D���̃`�F�b�N���K�v�Ȃ̂́C�^�X�N��O��
	 *  �����[�`���̌ďo�����K�v�ȏꍇ�ɁC�f�B�X�p�b�`���K�v�Ȃ��Ă��C
	 *  reqflg��true�ɂ��邽�߂ł���D
	 */
	ldr   r0, =p_runtsk       /* �f�B�X�p�b�`���s��Ȃ��ꍇ�ł��Cr1��p_runtsk �̒l(TCB) */
	ldr   r1, [r0]            /* �������Ă���K�v������̂ŁC��ɓǂݍ��� */
	ldr   r0, =dspflg         
	ldr   r2, [r0]
	cmp   r2, #0x01           /* dspflg��false�Ȃ�ret_int_r_1�� */
	bne   ret_int_r_1     
	ldr   r0, =p_schedtsk     
	ldr   r2, [r0]
	cmp   r1, r2              /* p_runtsk��p_schedtsk�������Ȃ� */
	beq   ret_int_r_1         /*                  ret_int_r_1�� */
	mov  r0, r8
	mov  r1, r9
	mov  r2, r10
	mov  r3, r11
	push {r0-r7}
	ldr   r2, =p_runtsk
	ldr   r1, [r2]
	mov   r0, sp
	str   r0, [r1,#TCB_sp]    /* �^�X�N�X�^�b�N��ۑ� */
	ldr   r2, =ret_int_r      /* ���s�ĊJ�Ԓn��ۑ�   */
	str   r2, [r1,#TCB_pc]
	b     dispatcher          /* �f�B�X�p�b�`���� */

/*
 * �����݂ɂ��v���G���v�g���ꂽ�^�X�N�ւ̃��^�[������
 *
 * Thread���[�h�ŁC�f�B�X�p�b�`���⊄���݂̏o����������Ăяo�����D
 * �����݂ɂ��v���G���v�g���ꂽ�^�X�N�փ��^�[������ɂ́C��������
 * Handler���[�h�Ɉڍs���CPC��0xfffffffd�������ă��^�[������K�v
 * ������D���̂��߁CSVC�ɂ��CSVC�n���h�����Ăяo���CHandler���[
 * �h�ֈڍs����D
 */
	ATHUMB(ret_int_r)
	AGLOBAL(ret_int_r)
ALABEL(ret_int_r)
	pop  {r0-r7}
	mov  r8,  r0
	mov  r9,  r1
	mov  r10, r2
	mov  r11, r3
ALABEL(ret_int_r_1)
	/*
	 *  enatex��true�ŁCtexptn��0�łȂ���΁C�^�X�N��O�������[�`����
	 *  �Ăяo���D
	 *  dispatcher����Ăяo����邽�߁CTCB�̃A�h���X��r1�ɓ����Ă���
	 */
	ldr   r2, =p_runtsk
	ldr   r1, [r2]
	ldrb  r0, [r1,#TCB_enatex]
	movs  r2, #TCB_enatex_mask
	tst   r0, r2
	beq   ret_int_r_2           /* enatex �� false �Ȃ� ret_int_r_2�� */
	ldr   r0, [r1,#TCB_texptn]  /* texptn �� 0 �Ȃ烊�^�[��     */
	cmp   r0, #0x00
	beq   ret_int_r_2
	ldr   r1, =ipmflg             /* ipmflg�� false �Ȃ烊�^�[��  */
	ldr   r0, [r1]
	cmp   r0, #0x00
	beq   ret_int_r_2 
	bl    call_texrtn           /* �^�X�N��O���[�`���̌Ăяo�� */
ALABEL(ret_int_r_2)
	ldr   r0, =NVIC_ICSR
	ldr   r1, =NVIC_PENDSVSET
	str   r1, [r0]
	cpsie i                     /* PRIMASK���N���A����PendSVC���󂯕t���� */                
1:                              /* �����ɂ͗��Ȃ� */
	b  1b

/*
 *  SVC�n���h��
 */
	ATHUMB(svc_handler)
	AGLOBAL(svc_handler)
ALABEL(svc_handler)
1:
	b  1b

/*
 *  PendSVC�n���h��
 */
	ATHUMB(pendsvc_handler)
	AGLOBAL(pendsvc_handler)
ALABEL(pendsvc_handler)
	/*
	 *  �����ݏ�������̃��^�[���ɂ��CCPU���b�N������ԂɈڍs�����
	 *  ����������D
	 */
	mrs   r0, psp
	adds  r0, #EXC_FRAME_SIZE     /* �X�^�b�N���̂Ă�   */
	msr   psp, r0
	movs  r0, #0
	bx    lr                      /* ���^�[��     */

/*
 *  �f�B�X�p�b�`���̓���J�n
 */
	ATHUMB(start_dispatch)
	AGLOBAL(start_dispatch)
ALABEL(start_dispatch)
	/*
	 *  ���̃��[�`���́C�J�[�l���N�����ɁC���ׂĂ̊����݂��֎~�������
	 * �i�����݃��b�N��ԂƓ����j�ŌĂяo�����D�܂��C�����݃��[�h�i��
	 *  �^�X�N�R���e�L�X�g�Ɠ����j�ŌĂяo����邱�Ƃ�z�肵�Ă���D
	 *
	 *  core_initialize�ŁClock_flag��true�ɁCsaved_iipm��IIPM_ENAALL��
	 *  ���������Ă��邽�߁C�J�[�l���Ǘ��O�̊����݂������邱�ƂŁC
	 *  CPU���b�N��ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂɂȂ�D
	 *  �܂��Ctask_initialize��disdsp��false�ɏ��������Ă��邽�߁C�f�B
	 *  �X�p�b�`����ԂɂȂ��Ă���D
	 */
	ldr   r0,=istkpt              /* MSP��������   */
	ldr   r1,[r0]                 /* start_dispatch�Ăяo�����ɌĂяo���p�� */
	msr   msp, r1                 /* �g�p���Ă��邽�ߏ���������             */
	movs  r0, #CONTROL_PSP        /* PSP��L����  */
	msr   control, r0
	isb                           /* control �̑����ɕK�v */

/*
 *  ���݂̃R���e�L�X�g���̂Ăăf�B�X�p�b�`
 */
	ATHUMB(exit_and_dispatch)
	AGLOBAL(exit_and_dispatch)
ALABEL(exit_and_dispatch)
	/* �f�B�X�p�b�`���{�́idispatcher�j�� */ 


/*
 *  �f�B�X�p�b�`���{��
 */
ALABEL(dispatcher)
	/*
	 *  ���̃��[�`���́C�^�X�N�R���e�L�X�g�ECPU���b�N��ԁE�f�B�X�p�b�`
	 *  ����ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂŌĂяo��
	 *  ���D
	 *
	 *  ���Ȃ킿�CThread���[�h�Elock_flag��true�Edisdsp��false�Edspflg
	 *  ��true�Esaved_iipm��IIPM_ENAALL�ƂȂ��Ă���D���s�ĊJ�Ԓn�ւ���
	 *  �̏�Ԃ̂܂ܕ��򂷂�D
	 */
#ifdef LOG_DSP_ENTER
	ldr   r1, =p_runtsk     /* p_runtsk���p�����[�^�� */
	ldr   r0, [r1]        
	bl    log_dsp_enter
#endif /* LOG_DSP_ENTER */
ALABEL(dispatcher_0)
	ldr   r0, =p_schedtsk   /* p_schedtsk��p_runtsk�� */
	ldr   r1, [r0]
	ldr   r2, =p_runtsk   
	str   r1, [r2]        
	cmp   r1, #0x00         /* p_runtsk��NULL�Ȃ�dispatcher_1�� */
	beq   dispatcher_1  
	ldr   r2, [r1,#TCB_sp]  /* �^�X�N�X�^�b�N�𕜋A */
	mov   sp, r2
#ifdef LOG_DSP_LEAVE
	mov   r0, r1            /* p_runtsk���p�����[�^�� */
	mov   r4, r1            /* r1�̓X�N���b�`���W�X�^�Ȃ̂ŕۑ� */
	bl    log_dsp_leave
	mov   r1, r4
#endif /* LOG_DSP_LEAVE */
	ldr   r2, [r1,#TCB_pc]  /* ���s�ĊJ�Ԓn�𕜋A   */    
	mov   pc, r2
ALABEL(dispatcher_1)
	/*
	 * CPU���b�N��Ԃ̉����ƁC��^�X�N�R���e�L�X�g���s��Ԃւ�
	 * ����������
	 */
	movs  r0, #CONTROL_MSP  /* MSP��L����  */
	msr   control, r0       
	isb                     /* control �̑����ɕK�v */
	movs  r4, #0            /* r4 <- '0' */
	ldr   r6, =reqflg       /* r6 <- reqflg */
ALABEL(dispatcher_2)
	/*
	 *  �����݂������C��^�X�N�R���e�L�X�g���s��ԂƂ������݂�҂D
	 *
	 *  �����Ŕ�^�X�N�R���e�L�X�g���s��Ԃɐ؂芷����̂́C�����Ŕ���
	 *  ���銄���ݏ����ɂǂ̃X�^�b�N���g�����Ƃ������̉����ƁC������
	 *  �n���h�����ł̃^�X�N�f�B�X�p�b�`�̖h�~�Ƃ���2�̈Ӗ�������D
	 *
	 *  �v���Z�b�T�������ݑ҂��Ɉڍs�����鏈���ƁC�����݋��Ƃ́C�s��
	 *  ���ɍs�Ȃ��K�v������D
	 *  �����s���ɍs�Ȃ�Ȃ��ꍇ�C�����݂�����������Ɋ���
	 *  �݂�����C���̒��Ń^�X�N�����s�\��ԂɂȂ�ƁC���s���ׂ��^�X
	 *  �N������ɂ�������炸�v���Z�b�T�������ݑ҂��ɂȂ��Ă��܂��D
	 *  ARM-M�ł́CPRIMASK���Z�b�g������Ԃ�WFI���Ăяo�����ƂŎ����ł���D
	 *  ���̏�ԂŊ����݂�����ƁC�����݂͎��s���ꂸ�CWFI���烊�^�[����
	 *  �邱�ƂɂȂ�̂ŁC��U�����݂������Ċ����݃n���h�������s����D
	 *
	 *  �����ݑ҂��̊Ԃ́Cp_runtsk��NULL�i��0�j�ɐݒ肵�Ȃ���΂Ȃ��
	 *  ���D���̂悤�ɐݒ肵�Ȃ��ƁC�����݃n���h������iget_tid���Ăяo
	 *  �����ۂ̓��삪�d�l�ɍ��v���Ȃ��Ȃ�D
	 *
	 *  �^�[�Q�b�g�ɂ���ẮC�ȓd�̓��[�h���Ɉڍs���邽�߁C�W���̕��@��
	 *  �قȂ�菇���K�v�ȏꍇ������D
	 *  ���̂悤�ȃ^�[�Q�b�g�ł́C�^�[�Q�b�g�ˑ��ɂ����āCTOPPERS_CUSTOM_IDLE
	 *  ���`���C�A�Z���u���}�N���Ƃ��āCtoppers_asm_custom_idle ��p��
	 *  ����΂悢�D
	 *
	 *  �Ȃ��Ctoppers_asm_custom_idle �̋L�q�ɂ������ẮC���̃��W�X�^��
	 *  toppers_asm_custom_idle�̑O��Ŏg�p���邽�߁C
	 *  toppers_asm_custom_idle ���Ŏg�p����ꍇ�́C�O��ŕۑ����A���邱�ƁD
	 *  �����̃��W�X�^�� Calee saved ���W�X�^�ł��邽�߁C 
	 *  toppers_asm_custom_idle �Ƃ��Ċ֐��Ăяo���������ꍇ�́C�Ăяo����
	 *  �֐��Ŏ����I�ɕۑ����A����邽�߁C�A�Z���u�����x���ł̕ۑ����A�͕K
	 *  �v�Ȃ��D
	 *
	 *  ���W�X�^ : ���e
	 *   r4      : '0'
	 *   r5      : 'IIPM_LOCK'
	 *   r6      : reqflg�̃A�h���X
	 *   r7      : lock_flg�̃A�h���X
	 *   sp      : ��^�X�N�R���e�L�X�g�p�̃X�^�b�N�̐擪�A�h���X(msp)
	 */
#ifdef TOPPERS_CUSTOM_IDLE
	toppers_asm_custom_idle
#else
	wfi
	cpsie i               /* PRIMASK ���N���A�i�����݂��󂯕t����j */
	cpsid i               /* PRIMASK ���Z�b�g */
#endif /* TOPPERS_CUSTOM_IDLE */

	ldr   r0, [r6]        /* reqflg��false�Ȃ�dispatcher_2�� */
	cmp   r0, #0
	beq   dispatcher_2
	str   r4, [r6]        /* reqflg��false�� */

	/*
	 *  CPU���b�N��Ԃɖ߂��D�����ݑ҂��̊ԂɎ��s���������݃n���h���ɂ�
	 *  ��Csaved_iipm��������������\�������邽�߁C���̒l�ɖ߂��K
	 *  �v������Ddispatcher�����s����鎞�́Csaved_iipm��IIPM_ENAALL
	 *  �ƂȂ��Ă��邽�߁C�����ł�saved_iipm��IIPM_ENAALL�i��0�j�ɖ߂�
	 *  �΂悢�D
	 */
	movs  r0, #CONTROL_PSP  /* PSP��L����  */ 
	msr   control, r0      
	isb                     /* control �̑����ɕK�v */
	b     dispatcher_0

/*
 *  �J�[�l���̏I�������̌ďo��
 *
 *  �X�^�b�N���^�X�N�R���e�L�X�g�p�ɐ؂�ւ��D
 *  
 */
	ATHUMB(call_exit_kernel)
	AGLOBAL(call_exit_kernel)
ALABEL(call_exit_kernel)
	movs  r0, #CONTROL_MSP
	msr   control, r0       /* MSP��L����  */
	isb                     /* control �̑����ɕK�v */
	ldr   r0, =exit_kernel  /* �J�[�l���̏I���������Ă� */
	bx    r0

/*
 *  �^�X�N�N������
 *
 *  dispatcher����Ăяo����邽�߁CTCB�̃A�h���X��r1�ɓ����Ă���
 *
 */ 
	ATHUMB(start_r)
	AGLOBAL(start_r)
ALABEL(start_r)
	movs  r0, #0
	cpsie i                                 /* �����݋���   */
	ldr   r0, =ext_tsk                      /* �߂�Ԓn�ݒ� */
	mov   lr, r0
	ldr   r2, [r1, #TCB_p_tinib]            /* p_runtsk->p_tinib��r2��  */
	ldr   r0, [r2, #TINIB_exinf]            /* exinf���������W�X�^r0��  */
	ldr   r1, [r2, #TINIB_task]             /* �^�X�N�N���Ԓn�ɃW�����v */
	bx    r1

/*
 *  �������ԑ҂�
 */
	ABALIGN(4)
	ATEXT
	ATHUMB(sil_dly_nse)
	AGLOBAL(sil_dly_nse)
ALABEL(sil_dly_nse)
	ldr   r1, =SIL_DLY_TIM1
	subs  r0, r0, r1
	cmp   r0, #0
	bgt   sil_dly_nse1
	bx    lr
ALABEL(sil_dly_nse1)
	ldr   r1, =SIL_DLY_TIM2
ALABEL(sil_dly_nse2)
	subs  r0, r0, r1
	cmp   r0, #0
	bgt   sil_dly_nse2
	bx    lr
