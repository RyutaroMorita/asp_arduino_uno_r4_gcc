/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2015 by Embedded and Real-Time Systems Laboratory
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
 */

#ifndef TOPPERS_CHIP_CONFIG_H
#define TOPPERS_CHIP_CONFIG_H

/*
 *  �^�[�Q�b�g�ˑ������W���[���iArduino UNO R4�p�j
 *
 *  �J�[�l���̃^�[�Q�b�g�ˑ����̃C���N���[�h�t�@�C���Dkernel_impl.h�̃^�[
 *  �Q�b�g�ˑ����̈ʒu�t���ƂȂ��D
 */

/*
 *  �g���[�X���O�Ɋւ���ݒ�
 */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  �G���[�`�F�b�N���@�̎w��
 */
#define CHECK_STKSZ_ALIGN	8	/* �X�^�b�N�T�C�Y�̃A���C���P�� */
#define CHECK_FUNC_ALIGN	1	/* �֐��̃A���C���P�� */
#define CHECK_FUNC_NONNULL		/* �֐��̔�NULL�`�F�b�N */
#define CHECK_STACK_ALIGN	8	/* �X�^�b�N�̈�̃A���C���P�� */
#define CHECK_STACK_NONNULL		/* �X�^�b�N�̈�̔�NULL�`�F�b�N */
#define CHECK_MPF_ALIGN		4	/* �Œ蒷�������v�[���̈�̃A���C���P�� */
#define CHECK_MPF_NONNULL		/* �Œ蒷�������v�[���̈�̔�NULL�`�F�b�N */
#define CHECK_MB_ALIGN		4	/* �Ǘ��̈�̃A���C���P�� */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  �v���Z�b�T�̓��ꖽ�߂̃C�����C���֐���`
 */
#include <core_insn.h>

/*
 *  ��^�X�N�R���e�L�X�g�p�̃X�^�b�N�����l
 */
#define TOPPERS_ISTKPT(istk, istksz) ((STK_T *)((uint8_t *)(istk) + (istksz)))

/*
 *  �^�X�N�R���e�L�X�g�u���b�N�̒�`
 */
typedef struct task_context_block {
	void	*sp;		/* �X�^�b�N�|�C���^ */
	FP		pc;			/* �v���O�����J�E���^ */
} TSKCTXB;

/*
 *  �R���e�L�X�g�̎Q��
 *
 */
Inline bool_t
sense_context(void)
{
	/*
	 *  PSP���L���Ȃ�^�X�N�R���e�L�X�g�CMSP���L���Ȃ��^�X�N�R���e�L�X�g
	 *  �Ƃ���D
	 */
	if ((get_control() & CONTROL_PSP) == CONTROL_PSP){
		return false;
	}
	else {
		return true;
	}
}

/*
 *  �X�^�[�g�A�b�v���[�`���ichip_start.S�j
 */
extern void _reset(void);

/*
 *  �ō��D�揇�ʃ^�X�N�ւ̃f�B�X�p�b�`�icore_support.S�j
 *
 *  dispatch�́C�^�X�N�R���e�L�X�g����Ăяo���ꂽ�T�[�r�X�R�[��������
 *  ��Ăяo���ׂ����̂ŁC�^�X�N�R���e�L�X�g�ECPU���b�N��ԁE�f�B�X�p�b
 *  �`����ԁE�i���f����́j�����ݗD��x�}�X�N�S������ԂŌĂяo����
 *  ����΂Ȃ�Ȃ��D
 */
extern void dispatch(void);

/*
 *  �f�B�X�p�b�`���̓���J�n�icore_support.S�j
 *
 *  start_dispatch�́C�J�[�l���N�����ɌĂяo���ׂ����̂ŁC���ׂĂ̊���
 *  �݂��֎~������ԁi�����݃��b�N��ԂƓ����̏�ԁj�ŌĂяo���Ȃ����
 *  �Ȃ�Ȃ��D
 */
extern void start_dispatch(void) NoReturn;

/*
 *  ���݂̃R���e�L�X�g���̂Ăăf�B�X�p�b�`�icore_support.S�j
 *
 *  exit_and_dispatch�́Cext_tsk����Ăяo���ׂ����̂ŁC�^�X�N�R���e�L
 *  �X�g�ECPU���b�N��ԁE�f�B�X�p�b�`����ԁE�i���f����́j�����ݗD��
 *  �x�}�X�N�S������ԂŌĂяo���Ȃ���΂Ȃ�Ȃ��D
 */
extern void exit_and_dispatch(void) NoReturn;

/*
 *  �J�[�l���̏I�������̌ďo���icore_support.S�j
 *
 *  call_exit_kernel�́C�J�[�l���̏I�����ɌĂяo���ׂ����̂ŁC��^�X�N
 *  �R���e�L�X�g�ɐ؂芷���āC�J�[�l���̏I�������iexit_kernel�j���Ăяo
 *  ���D
 */
extern void call_exit_kernel(void) NoReturn;

/*
 *  �^�X�N�R���e�L�X�g�̏�����
 *
 *  �^�X�N���x�~��Ԃ�����s�ł����ԂɈڍs���鎞�ɌĂ΂��D���̎��_
 *  �ŃX�^�b�N�̈���g���Ă͂Ȃ�Ȃ��D
 *
 *  activate_context���C�C�����C���֐��ł͂Ȃ��}�N����`�Ƃ��Ă���̂́C
 *  ���̎��_�ł�TCB����`����Ă��Ȃ����߂ł���D
 */
extern void start_r(void);

#define activate_context(p_tcb)											\
{																		\
	(p_tcb)->tskctxb.sp = (void *)((uint8_t *)((p_tcb)->p_tinib->stk)	\
								+ (p_tcb)->p_tinib->stksz);				\
	(p_tcb)->tskctxb.pc = (FP) start_r;									\
}

/*
 *  calltex�͎g�p���Ȃ�
 */
#define OMIT_CALLTEX

/*
 *  �����ݔԍ��E�����݃n���h���ԍ�
 *
 *  �����݃n���h���ԍ�(inhno)�Ɗ����ݔԍ�(intno)�́C���荞�ݔ�������
 *  IPSR�ɐݒ肳����O�ԍ��Ƃ���D
 */

/*
 *  �����ݔԍ��͈̔͂̔���
 */
#define VALID_INTNO(intno)           ((TMIN_INTNO <= (intno)) && ((intno) <= TMAX_INTNO))
#define VALID_INTNO_DISINT(intno)    VALID_INTNO(intno)
#define VALID_INTNO_CFGINT(intno)    VALID_INTNO(intno)

/*
 *  �����݃n���h���̐ݒ�
 *
 *  �x�N�g���ԍ�inhno�̊����݃n���h���̋N���Ԓnint_entry�ɐݒ肷��D������
 *  �n���h���e�[�u��
 */
Inline void
x_define_inh(INHNO inhno, FP int_entry)
{

}

/*
 *  �����݃n���h���̏o���������̐����}�N��
 *
 */
#define INT_ENTRY(inhno, inthdr)    inthdr
#define INTHDR_ENTRY(inhno, inhno_num, inthdr) extern void inthdr(void);

/*
 *  �����ݗv�����C���̑����̐ݒ�
 */
extern void x_config_int(INTNO intno, ATR intatr, PRI intpri);

/*
 *  �����݃n���h�������ŕK�v��IRC����
 */
Inline void
i_begin_int(INTNO intno)
{
}

/*
 *  �����݃n���h���̏o���ŕK�v��IRC����
 */
Inline void
i_end_int(INTNO intno)
{
}

/*
 *  CPU��O�G���g���icore_support.S�j
 */
extern void core_exc_entry(void);

/*
 *  �����݃G���g���icore_support.S�j
 */
extern void core_int_entry(void);

/*
 *  �v���Z�b�T�ˑ��̏�����
 */
extern void core_initialize(void);

/*
 *  �v���Z�b�T�ˑ��̏I��������
 */
extern void core_terminate(void);

/*
 * �o�^����Ă��Ȃ���O����������ƌĂяo�����
 */
extern void default_exc_handler(void *p_excinf);

/*
 * ���o�^�̊����݂����������ꍇ�ɌĂяo�����
 */
extern void default_int_handler(void *p_excinf);

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  ARMv7-M��ARMv6-M�ňقȂ鏈��
 *  ARMv6-M�̏�����core_config_armv6m.h�ɋL�q����
 */
#if __TARGET_ARCH_THUMB == 4

/*
 *  ARMv7-M�Ɋւ��鏈��
 */

/*
 *  �^�[�Q�b�g�ˑ��̃I�u�W�F�N�g����
 */
#define TARGET_INHATR  TA_NONKERNEL /* �^�[�Q�b�g��`�̊����݃n���h������ */

/*
 *  TOPPERS�W�������ݏ������f���̎���
 *
 *  �����ݗD��x�}�X�N�Ƃ��ẮCBASEPRI��p����D�S�����݂��֎~����
 *  �@�\�Ƃ��āCFAULTMASK��PRIMASK�����邪�C�J�[�l���Ǘ��O�̊����݂�
 *  �T�|�[�g���邽�߁C������CPU���b�N�̂��߂ɗp���Ȃ��D
 *  ���̂��߁CBASEPRI��p���ċ[���I��CPU���b�N�t���O����������D
 *
 *  �܂��CCPU���b�N��Ԃ��Ǘ������߂̕ϐ�(lock_flag)��p�ӂ���D
 *
 *  CPU���b�N�t���O���N���A����Ă���Ԃ́CBASEPRI�����f����̊�����
 *  �D��x�}�X�N�̒l�ɐݒ肷��D���̊Ԃ́C���f����̊����ݗD��x�}�X
 *  �N�́CBASEPRI��p����D
 *
 *  ����ɑ΂���CPU���b�N�t���O���Z�b�g���ꂢ��Ԃ́CBASEPRI���C�J�[�l
 *  ���Ǘ��O�̂��̂��������ׂĂ̊����ݗv�����}�X�N����l(TIPM_LOCK)�ƁC
 *  ���f����̊����ݗD��x�}�X�N�Ƃ̍������ɐݒ肷��D���̊Ԃ̃��f����
 *  �̊����ݗD��x�}�X�N�́C���̂��߂̕ϐ�(saved_iipm, �����\���ŕێ�)
 *  ��p�ӂ��ĕێ�����D
 */

/*
 *  �����ݗD��x�}�X�N�̊O���\���Ɠ����\���̕ϊ�
 *
 *  �A�Z���u������̃\�[�X�t�@�C������C���N���[�h����ꍇ�̂��߂ɁC
 *  CAST���g�p
 *  �����ݗD��x�̃r�b�g��(TBITW_IPRI)�� 8 �̏ꍇ�́C�����D��x 255
 *  �́C�O���D��x -1 �ɑΉ�����D
 */
#define EXT_IPM(iipm)   (CAST(PRI,((iipm >> (8 - TBITW_IPRI)) - (1 << TBITW_IPRI))))       /* �����\�����O���\���� */
#define INT_IPM(ipm)    (((1 << TBITW_IPRI) - CAST(uint8_t, -(ipm)))  << (8 - TBITW_IPRI)) /* �O���\��������\���� */

/*
 *  �����ݗD��x�}�X�N��NVIC�̗D��x�ɕϊ�
 */
#define INT_NVIC_PRI(ipm)    INT_IPM(ipm)

/*
 *  CPU���b�N��Ԃł̊����ݗD��x�}�X�N
 */
#define TIPM_LOCK    TMIN_INTPRI

/*
 *  CPU���b�N��Ԃł̊����ݗD��x�}�X�N�̓����\��
 *
 *  TIPM_LOCK�́CCPU���b�N��Ԃł�BASEPRI�̒l�D�J�[�l���Ǘ��O�̂��̂�
 *  �������ׂĂ̊����݂��}�X�N����l�ɒ�`����D
 */
#define IIPM_LOCK    INT_IPM(TIPM_LOCK)

/*
 *  TIPM_ENAALL�i�����ݗD��x�}�X�N�S�����j�̓����\��
 *
 *  BASEPRI�� '0' ��ݒ肷�邱�ƂŁC�S�����݂�������D
 */
#define IIPM_ENAALL  (0)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPU���b�N�t���O�����̂��߂̕ϐ�
 *
 *  �����̕ϐ��́CCPU���b�N��Ԃ̎��̂ݏ��������Ă��悢�Ƃ���D
 *  �C�����C���֐����ŁC�A�N�Z�X�̏������ω����Ȃ��悤�Cvolatile ���w��D
 */
extern volatile bool_t  lock_flag;    /* CPU���b�N�t���O�̒l��ێ�����ϐ� */
extern volatile uint32_t saved_iipm;  /* �����ݗD��x���}�X�N����ϐ� */

/*
 *  CPU���b�N��Ԃւ̈ڍs
 *
 *  BASEPRI�i�n�[�h�E�F�A�̊����ݗD��x�}�X�N�j���Csaved_iipm�ɕۑ����C
 *  �J�[�l���Ǘ��O�̂��̂��������ׂĂ̊����݂��}�X�N����l�iTIPM_LOCK�j
 *  �ɐݒ肷��D�܂��Clock_flag��true�ɂ���D
 *
 *  BASEPRI���C�ŏ�����TIPM_LOCK�Ɠ����������荂���ꍇ�ɂ́C�����
 *  saved_iipm�ɕۑ�����݂̂ŁCTIPM_LOCK�ɂ͐ݒ肵�Ȃ��D����́C���f��
 *  ��̊����ݗD��x�}�X�N���CTIPM_LOCK�Ɠ����������荂�����x���ɐݒ�
 *  ����Ă����Ԃɂ�����D
 *
 *  ���̊֐��́CCPU���b�N��ԁilock_flag��true�̏�ԁj�ŌĂ΂�邱�Ƃ�
 *  �Ȃ����̂Ƒz�肵�Ă���D
 */
Inline void
x_lock_cpu(void)
{
	uint32_t iipm;

	/*
	 *  get_basepri()�̕Ԃ�l�𒼐�saved_iipm�ɕۑ������C�ꎞ�ϐ�iipm
	 *  ��p���Ă���̂́Cget_baespri()���Ă񂾒���Ɋ����݂��������C
	 *  �N�����ꂽ�����ݏ�����saved_iipm���ύX�����\�������邽�߂�
	 *  ����D
	 */
	iipm = get_basepri();
	/*
	 *  BASEPRI���W�X�^�͒l���������قǗD��x���������CIIPM_ENAALL ��
	 *  '0'�ł��邽�߁C�P���ɗD��x��r�����ł͕s�\���ł���D
	 */
	if ((IIPM_LOCK < iipm) || (IIPM_ENAALL == iipm)) {
		set_basepri(IIPM_LOCK);
	}
	saved_iipm = iipm;
	lock_flag = true;

	/* �N���e�B�J���Z�N�V�����̑O��Ń����������������\�������� */
	ARM_MEMORY_CHANGED;
}

#define t_lock_cpu()    x_lock_cpu()
#define i_lock_cpu()    x_lock_cpu()

/*
 *  CPU���b�N��Ԃ̉���
 *
 *  lock_flag��false�ɂ��CIPM�i�n�[�h�E�F�A�̊����ݗD��x�}�X�N�j���C
 *  saved_iipm�ɕۑ������l�ɖ߂��D
 *
 *  ���̊֐��́CCPU���b�N��ԁilock_flag��true�̏�ԁj�ł̂݌Ă΂���
 *  �̂Ƒz�肵�Ă���D
 */
Inline void
x_unlock_cpu(void)
{
	/* �N���e�B�J���Z�N�V�����̑O��Ń����������������\�������� */
	ARM_MEMORY_CHANGED;
	lock_flag = false;
	set_basepri(saved_iipm);
}

#define t_unlock_cpu()    x_unlock_cpu()
#define i_unlock_cpu()    x_unlock_cpu()

/*
 *  CPU���b�N��Ԃ̎Q��
 */
Inline bool_t
x_sense_lock(void)
{
	return(lock_flag);
}

#define t_sense_lock()    x_sense_lock()
#define i_sense_lock()    x_sense_lock()

/*
 *  chg_ipm�ŗL���Ȋ����ݗD��x�͈̔͂̔���
 *
 *  TMIN_INTPRI�̒l�ɂ�炸�Cchg_ipm�ł́C-(1 << TBITW_IPRI)�`TIPM_ENAALL�i��0�j
 *  �͈̔͂ɐݒ�ł��邱�ƂƂ���i�^�[�Q�b�g��`�̊g���j�D
 *  �����ݗD��x�̃r�b�g��(TBITW_IPRI)�� 8 �̏ꍇ�́C-256 �` 0 ���w��\�ł���D
 *
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				((-((1 << TBITW_IPRI) - 1) <= (intpri) && (intpri) <= TIPM_ENAALL))

/*
 * �i���f����́j�����ݗD��x�}�X�N�̐ݒ�
 *
 *  CPU���b�N�t���O���N���A����Ă��鎞�́C�n�[�h�E�F�A�̊����ݗD��x�}
 *  �X�N��ݒ肷��DCPU���b�N�t���O���Z�b�g����Ă��鎞�́Csaved_iipm
 *  ��ݒ肵�C����ɁC�n�[�h�E�F�A�̊����ݗD��x�}�X�N���C�ݒ肵�悤��
 *  �����i���f����́j�����ݗD��x�}�X�N��TIPM_LOCK�̍������ɐݒ肷��D
 */
Inline void
x_set_ipm(PRI intpri)
{
	uint8_t   iipm = INT_IPM(intpri);

	if (intpri == TIPM_ENAALL){
		iipm = IIPM_ENAALL;
	}

	if (!lock_flag) {
		set_basepri(iipm);
	}
	else {
		saved_iipm = iipm;
		/*
		 *  BASEPRI���W�X�^�͒l���������قǗD��x���������CIIPM_ENAALL ��
		 *  '0'�ł��邽�߁C�P���ɗD��x��r�����ł͕s�\���ł���D
		 */
		if ((iipm < IIPM_LOCK ) && (IIPM_ENAALL != iipm)) {
			set_basepri(iipm);
		}
		else {
			set_basepri(IIPM_LOCK);
		}
	}
}

#define t_set_ipm(intpri)    x_set_ipm(intpri)
#define i_set_ipm(intpri)    x_set_ipm(intpri)

/*
 * �i���f����́j�����ݗD��x�}�X�N�̎Q��
 *
 *  CPU���b�N�t���O���N���A����Ă��鎞�̓n�[�h�E�F�A�̊����ݗD��x�}
 *  �X�N���C�Z�b�g����Ă��鎞��saved_iipm���Q�Ƃ���D
 */
Inline PRI
x_get_ipm(void)
{
	uint8_t iipm;

	if (!lock_flag) {
		iipm = get_basepri();
	}
	else {
		iipm = saved_iipm;
	}

	if (iipm == IIPM_ENAALL) {
		return(TIPM_ENAALL);
	}
	else {
		return(EXT_IPM(iipm));
	}
}

#define t_get_ipm()    x_get_ipm()
#define i_get_ipm()    x_get_ipm()

/*
 *  �����ݗv���֎~�t���O
 */

/*
 *  �����ݑ������ݒ肳��Ă��邩�𔻕ʂ��邽�߂̕ϐ��ikernel_cfg.c�j
 */
extern const uint32_t	bitpat_cfgint[];

/*
 *  �����ݗv���֎~�t���O�̃Z�b�g
 *
 *  �����ݑ������ݒ肳��Ă��Ȃ������ݗv�����C���ɑ΂��Ċ����ݗv���֎~
 *  �t���O���N���A���悤�Ƃ����ꍇ�ɂ́Cfalse��Ԃ��D
 */
Inline bool_t
x_disable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  �����ݑ������ݒ肳��Ă��Ȃ��ꍇ
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
		tmp &= ~SYSTIC_TICINT;
		sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)((uint32_t *)NVIC_CLRENA0 + (tmp >> 5)),
					(1 << (tmp & 0x1f)));
	}

	return(true);
}

#define t_disable_int(intno) x_disable_int(intno)
#define i_disable_int(intno) x_disable_int(intno)

/*
 *  �����ݗv���֎~�t���O�̉���
 *
 *  �����ݑ������ݒ肳��Ă��Ȃ������ݗv�����C���ɑ΂��Ċ����ݗv���֎~
 *  �t���O���N���A���悤�Ƃ����ꍇ�ɂ́Cfalse��Ԃ��D
 */
Inline bool_t
x_enable_int(INTNO intno)
{
	uint32_t tmp;

	/*
	 *  �����ݑ������ݒ肳��Ă��Ȃ��ꍇ
	 */
	if ((bitpat_cfgint[intno >> 5] & (1 << (intno & 0x1f))) == 0x00) {
		return(false);
	}

	if (intno == IRQNO_SYSTICK) {
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
		tmp |= SYSTIC_TICINT;
		sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)((uint32_t *)NVIC_SETENA0 + (tmp >> 5)),
					(1 << (tmp & 0x1f)));
	}
	return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)

/*
 *  SVC�n���h���icore_support.S�j
 */
extern void svc_handler(void);

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  FPU�֘A�̒�`
 */

/*
 *  FPCCR�̏����l
 */
#if defined(TOPPERS_FPU_NO_PRESERV)
#define FPCCR_INIT FPCCR_NO_PRESERV
#elif defined(TOPPERS_FPU_NO_LAZYSTACKING)
#define FPCCR_INIT FPCCR_NO_LAZYSTACKING
#elif defined(TOPPERS_FPU_LAZYSTACKING)
#define FPCCR_INIT FPCCR_LAZYSTACKING
#endif /* defined(TOPPERS_FPU_NO_PRESERV) */

#else /* __TARGET_ARCH_THUMB == 3 */

/*
 *  ARMv6-M�Ɋւ��鏈��
 */
#include "core_config_v6m.h"

#endif /* __TARGET_ARCH_THUMB == 4 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  CPU��O�n���h���֌W
 */

/*
 *  CPU��O�n���h���ԍ�
 */
#define VALID_EXCNO_DEFEXC(excno)    (TMIN_EXCNO <= (excno) && (excno) <= TMAX_EXCNO)

/*
 *  CPU��O�n���h���̋���
 */
extern void enable_exc(EXCNO excno);

/*
 *  CPU��O�n���h���̋֎~
 */
extern void disable_exc(EXCNO excno);

/*
 *  CPU��O�n���h���̐ݒ�
 */
Inline void
x_define_exc(EXCNO excno, FP exc_entry)
{
	/*
	 *  �ꕔ�̗�O�͋����s���K�v������
	 */
	enable_exc(excno);
}

/*
 *  CPU��O�n���h���̓��������̐����}�N��
 */
#define EXC_ENTRY(excno, exchdr)    exchdr
#define EXCHDR_ENTRY(excno, excno_num, exchdr) extern void exchdr(void *p_excinf);

/*
 *  CPU��O�̔����������̃R���e�L�X�g�̎Q��
 *
 *  CPU��O�̔����������̃R���e�L�X�g���C�^�X�N�R���e�L�X�g�̎���false�C
 *  �����łȂ�����true��Ԃ��D
 */
Inline bool_t
exc_sense_context(void *p_excinf)
{
	uint32_t exc_return;

	exc_return = *((uint32_t *)p_excinf + P_EXCINF_OFFSET_EXC_RETURN);
	if ((exc_return & EXC_RETURN_PSP) == EXC_RETURN_PSP){
		return false;
	}
	else {
		return true;
	}
}

/*
 *  CPU��O�̔�����������IPM�i�n�[�h�E�F�A�̊����ݗD��x�}�X�N�C�����\
 *  ���j�̎Q��
 */
Inline uint32_t
exc_get_iipm(void *p_excinf)
{
	return(*((uint32_t *)p_excinf + P_EXCINF_OFFSET_IIPM));
}

/*
 *  CPU��O�̔����������̃R���e�L�X�g�Ɗ����݂̃}�X�N��Ԃ̎Q��
 *
 *  CPU��O�̔����������̃V�X�e����Ԃ��C�J�[�l�����s���łȂ��C�^�X�N�R
 *  ���e�L�X�g�ł���C�����݃��b�N��ԂłȂ��CCPU���b�N��ԂłȂ��C�i��
 *  �f����́j�����ݗD��x�}�X�N�S������Ԃł��鎞��true�C�����łȂ���
 *  ��false��Ԃ��iCPU��O���J�[�l���Ǘ��O�̊����ݏ������Ŕ��������ꍇ
 *  �ɂ�false��Ԃ��j�D
 *
 *  PU��O�̔�����������BASEPRI�i�n�[�h�E�F�A�̊����ݗD��x�}�X�N�j
 *  �����ׂĂ̊����݂��������Ԃł��邱�Ƃ��`�F�b�N���邱�ƂŁC�J�[
 *  �l�����s���łȂ����ƁC�����݃��b�N��ԂłȂ����ƁCCPU���b�N��Ԃł�
 *  �����ƁC�i���f����́j�����ݗD��x�}�X�N�S������Ԃł��邱�Ƃ�4��
 *  �������`�F�b�N���邱�Ƃ��ł���iCPU��O��������������lock_flag���Q
 *  �Ƃ���K�v�͂Ȃ��j�D
 */
Inline bool_t
exc_sense_intmask(void *p_excinf)
{
	return(!exc_sense_context(p_excinf)
		   && (exc_get_iipm(p_excinf) == IIPM_ENAALL) && !x_sense_lock());
}
#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_CHIP_CONFIG_H */
