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
 *		�R�A�ˑ����W���[���iARM-M�p�j
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  TOPPERS�W�������ݏ������f�������̂��߂̕ϐ��Ə���������
 *  ARMv6-M��ARMv7-M�ňقȂ邽��ifdef�ɐ؂蕪���Ă���
 */

#if __TARGET_ARCH_THUMB == 4

volatile bool_t		lock_flag;		/* CPU���b�N�t���O�̒l��ێ�����ϐ� */
volatile uint32_t	saved_iipm;		/* �����ݗD��x�}�X�N��ۑ�����ϐ� */

static void
init_intmodel(void){
	lock_flag = true;
	saved_iipm = IIPM_ENAALL;
}

#else /* __TARGET_ARCH_THUMB == 3 */

uint32_t ief;			/* IRQ�̊����ݗv���֎~�t���O�̏�� */
uint8_t  ief_systick;	/* SysTick�̊����ݗv���֎~�t���O�̏�� */
uint8_t  iipm;			/* ���݂̊����ݗD��x�}�X�N�̒l */

static void
init_intmodel(void){
	ief = 0x00;
	ief_systick = 0x00;
	iipm = INT_IPM(0);
}

#endif /* __TARGET_ARCH_THUMB == 4 */

/*
 *  �x�N�^�e�[�u��(kernel_cfg.c)
 */
extern const FP vector_table[];

/*
 *  �V�X�e����O�E�����݂́i��O�ԍ� 4�`15�j
 *  �����ݗD��x�ݒ背�W�X�^�ւ̃A�N�Z�X�̂��߂̔z��
 */
static const unsigned int nvic_sys_pri_reg[] = {
	0,
	NVIC_SYS_PRI1,
	NVIC_SYS_PRI2,
	NVIC_SYS_PRI3
};

/*
 *  ��O�Ɗ����݂̊����ݗD��x���Z�b�g
 *
 *  excno��ARM-M�Œ�߂��Ă��� Exception Number ���w��D
 */
void
set_exc_int_priority(uint32_t excno, uint32_t nvic_ipm){
	uint32_t tmp, reg;

	/*
	 *  �����ݗD��x�ݒ背�W�X�^�̌���
	 */
	if ((EXCNO_MPU <= excno) && (excno <= IRQNO_SYSTICK)) {
		/*
		 * Exception Number 4(Memory Management)����
		 * Exception Number 15(SysTick)�܂ł̊����ݗD��x�̓V�X�e���D��x
		 * ���W�X�^�ɂ��ݒ�D
		 */
		reg = nvic_sys_pri_reg[excno >> 2];
	}
	else if ((TMIN_INTNO < excno) && (excno <= TMAX_INTNO)){
		/*
		 * IRQ�����݂Ȃ�
		 */
		reg = NVIC_PRI0 + (((excno - (TMIN_INTNO + 1)) >> 2) * 4);
	}
	else {
		return ;
	}

	tmp = sil_rew_mem((void *)reg);
	tmp &= ~(0xFF << (8 * (excno & 0x03)));
	tmp |= nvic_ipm << (8 * (excno & 0x03));
	sil_wrw_mem((void *)reg, tmp);
}

/*
 *  ��O�̋���
 *
 *  Memory Management, Bus Fault, Usage Fault �͋֎~�E�����\
 */
void
enable_exc(EXCNO excno)
{
	uint32_t tmp;

	switch (excno) {
	  case EXCNO_MPU:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp |= NVIC_SYS_HND_CTRL_MEM;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	  case EXCNO_BUS:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp |= NVIC_SYS_HND_CTRL_BUS;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	  case EXCNO_USAGE:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp |= NVIC_SYS_HND_CTRL_USAGE;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	}
}

/*
 *  ��O�̋֎~
 */
void
disable_exc(EXCNO excno)
{
	uint32_t tmp;

	switch (excno) {
	  case EXCNO_MPU:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp &= ~NVIC_SYS_HND_CTRL_MEM;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	  case EXCNO_BUS:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp &= ~NVIC_SYS_HND_CTRL_BUS;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	  case EXCNO_USAGE:
		tmp = sil_rew_mem((void *)NVIC_SYS_HND_CTRL);
		tmp &= ~NVIC_SYS_HND_CTRL_USAGE;
		sil_wrw_mem((void *)NVIC_SYS_HND_CTRL, tmp);
		break;
	}
}


/*
 *  �R�A�ˑ��̏�����
 */
void
core_initialize(void)
{
	/*
	 *  �x�N�^�e�[�u����ݒ�
	 */
	sil_wrw_mem((void*)NVIC_VECTTBL, (uint32_t)vector_table);

	/*
	 *  �e��O�̗D��x��ݒ�
	 *  CPU���b�N��Ԃł���������悤�ɁCBASEPRI���W�X�^�Ń}�X�N�ł�
	 *  �Ȃ�'0'�Ƃ���D
	 */
	set_exc_int_priority(EXCNO_HARD, 0);
	set_exc_int_priority(EXCNO_MPU, 0);
	set_exc_int_priority(EXCNO_BUS, 0);
	set_exc_int_priority(EXCNO_USAGE, 0);
	set_exc_int_priority(EXCNO_SVCALL, 0);
	set_exc_int_priority(EXCNO_DEBUG, 0);
	set_exc_int_priority(EXCNO_PENDSV, 0);

	/*
	 *  �����ݏ������f���֘A�̏�����
	 */
	init_intmodel();
}

/*
 *  �R�A�ˑ��̏I������
 */
void
core_terminate(void)
{
	extern void    software_term_hook(void);
	void (*volatile fp)(void) = software_term_hook;

	/*
	 *  software_term_hook�ւ̃|�C���^���C��Uvolatile�w��̂���fp�ɑ�
	 *  �����Ă���g���̂́C0�Ƃ̔�r���œK���ō폜����Ȃ��悤�ɂ��邽
	 *  �߂ł���D
	 */
	if (fp != 0) {
		(*fp)();
	}
}

/*
 *  �����ݗv�����C�������̐ݒ�
 */
void
x_config_int(INTNO intno, ATR intatr, PRI intpri)
{
	assert(VALID_INTNO_CFGINT(intno));
	assert(TMIN_INTPRI <= intpri && intpri <= TMAX_INTPRI);

	/* 
	 *  ��U�����݂��֎~����
	 */    
	(void)x_disable_int(intno);

	/*
	 *  �����ݗD��x���Z�b�g
	 */
	set_exc_int_priority(intno, INT_NVIC_PRI(intpri));

	/*
	 *  �����ݗv���}�X�N����(�K�v�ȏꍇ)
	 */
	if ((intatr & TA_ENAINT) != 0U) {
		(void) x_enable_int(intno);
	}    
}


#ifndef OMIT_DEFAULT_EXC_HANDLER
/*
 *  ����`�̗�O����������ƌĂяo�����
 */
void
default_exc_handler(void *p_excinf)
{
	uint32_t basepri = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_IIPM);
	uint32_t pc      = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_PC);
	uint32_t xpsr    = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_XPSR);
	uint32_t excno   = get_ipsr() & IPSR_ISR_NUMBER;

	syslog(LOG_EMERG, "\nUnregistered Exception occurs.");
	syslog(LOG_EMERG, "Excno = 0x%08X, PC = 0x%08X, XPSR = 0x%08X, iipm = 0x%08X, p_excinf = 0x%08X",
		   excno, pc, xpsr, basepri, p_excinf);	

	target_exit();
}
#endif /* OMIT_DEFAULT_EXC_HANDLER */

#ifndef OMIT_DEFAULT_INT_HANDLER
/*
 *  ���o�^�̊����݂����������ꍇ�ɌĂяo�����
 */
void
default_int_handler(void *p_excinf)
{
	uint32_t basepri = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_IIPM);
	uint32_t pc      = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_PC);
	uint32_t xpsr    = *(((uint32_t*)p_excinf) + P_EXCINF_OFFSET_XPSR);
	uint32_t excno   = get_ipsr() & IPSR_ISR_NUMBER;

	syslog(LOG_EMERG, "\nUnregistered Interrupt occurs.");
	syslog(LOG_EMERG, "Excno = 0x%08X, PC = 0x%08X, XPSR = 0x%08X, iipm = 0x%08X, p_excinf = 0x%08X",
		   excno, pc, xpsr, basepri, p_excinf);	

	target_exit();
}
#endif /* OMIT_DEFAULT_INT_HANDLER */
