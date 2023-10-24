/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2015 by Embedded and Real-Time Systems Laboratory
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
 *		�����ݏ������f���iARMv6-M�p�j
 *
 *  ���̃C���N���[�h�t�@�C���́Ccore_config.h�i�܂��́C��������C���N
 *  ���[�h�����t�@�C���j�݂̂���C���N���[�h�����D���̃t�@�C������
 *  ���ڃC���N���[�h���Ă͂Ȃ�Ȃ��D
 */

#ifndef TOPPERS_CORE_INTMODEL_V6M_H
#define TOPPERS_CORE_INTMODEL_V6M_H

/*
 *  �^�[�Q�b�g�ˑ��̃I�u�W�F�N�g����
 */
#define TARGET_INHATR  TA_NONKERNEL /* �^�[�Q�b�g��`�̊����݃n���h������ */

/*
 *  �����ݗD��x�}�X�N�̊O���\���Ɠ����\���̕ϊ�
 *
 *  �A�Z���u������̃\�[�X�t�@�C������C���N���[�h����ꍇ�̂��߂ɁC
 *  CAST���g�p
 *   �O���\�� : TMIN_INTPRI  �`       0
 *   �����\�� :      0       �`  -TMIN_INTPRI
 */
#define EXT_IPM(iipm)   (CAST(PRI,iipm + TMIN_INTPRI))     /* �����\�����O���\���� */
#define INT_IPM(ipm)    (CAST(uint8_t, ipm - TMIN_INTPRI)) /* �O���\��������\���� */

/*
 *  �����ݗD��x�}�X�N��NVIC�̗D��x�ɕϊ�
 */
#define INT_NVIC_PRI(ipm)    (((1 << TBITW_IPRI) - CAST(uint8_t, -(ipm)))  << (8 - TBITW_IPRI))

/*
 *  TIPM_ENAALL�i�����ݗD��x�}�X�N�S�����j�̓����\��
 *
 */
#define IIPM_ENAALL  (-TMIN_INTPRI)

#ifndef TOPPERS_MACRO_ONLY

/*
 *  �����ݗv���֎~�t���O�̎����̂��߂̕ϐ�
 */
extern uint32_t ief;			/* IRQ�̊����ݗv�����t���O�̏�� */
extern uint8_t  ief_systick;	/* SysTick�̊����ݗv�����t���O�̏�� */

/*
 *  �����ݗD��x�}�X�N�����̂��߂̕ϐ�
 */
extern uint8_t iipm;		/* ���݂̊����ݗD��x�}�X�N�̒l */

/*
 *  �����ݗD��x�}�X�N�����̂��߂̕ϐ��ikernel_cfg.c�j
 */
extern const uint32_t iipm_enable_irq_tbl[];
extern const uint8_t iipm_enable_systic_tbl[];

/*
 *  CPU���b�N��Ԃւ̈ڍs
 *
 */
Inline void
x_lock_cpu(void)
{
	set_primask();
	/* �N���e�B�J���Z�N�V�����̑O��Ń����������������\�������� */
	ARM_MEMORY_CHANGED;    
}

#define t_lock_cpu()    x_lock_cpu()
#define i_lock_cpu()    x_lock_cpu()

/*
 *  CPU���b�N��Ԃ̉���
 *
 */
Inline void
x_unlock_cpu(void)
{
	/* �N���e�B�J���Z�N�V�����̑O��Ń����������������\�������� */
	ARM_MEMORY_CHANGED;
	clear_primask();
}

#define t_unlock_cpu()    x_unlock_cpu()
#define i_unlock_cpu()    x_unlock_cpu()

/*
 *  CPU���b�N��Ԃ̎Q��
 */
Inline bool_t
x_sense_lock(void)
{
	return(read_primask() == 0x1u);
}

#define t_sense_lock()    x_sense_lock()
#define i_sense_lock()    x_sense_lock()

/*
 *  chg_ipm�ŗL���Ȋ����ݗD��x�͈̔͂̔���
 *
 *  TMIN_INTPRI�̒l�ɂ�炸�Cchg_ipm�ł́C-(1 << TBITW_IPRI)�`TIPM_ENAALL�i��0�j
 *  �͈̔͂ɐݒ�ł��邱�ƂƂ���i�^�[�Q�b�g��`�̊g���j�D
 *  �����ݗD��x�̃r�b�g��(TBITW_IPRI)�� 2 �̏ꍇ�́C-4 �` 0 ���w��\�ł���D
 *   
 */
#define VALID_INTPRI_CHGIPM(intpri) \
				((-((1 << TBITW_IPRI) - 1) <= (intpri) && (intpri) <= TIPM_ENAALL))

/*
 * �i���f����́j�����ݗD��x�}�X�N�̐ݒ�
 *
 */
Inline void
x_set_ipm(PRI intpri)
{
	uint32_t tmp;
	iipm = INT_IPM(intpri);

	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	if ((iipm_enable_systic_tbl[iipm] & ief_systick) ==  0x01) {
		tmp |= SYSTIC_TICINT;
	}else{
		tmp &= ~SYSTIC_TICINT;
	}
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);

	/* ��U�S�����݋֎~ */
	sil_wrw_mem((void *)NVIC_CLRENA0, 0xffffffff);
//	sil_wrw_mem((void *)NVIC_SETENA0, (iipm_enable_systic_tbl[iipm] & ief));
	sil_wrw_mem((void *)NVIC_SETENA0, (iipm_enable_irq_tbl[iipm] & ief));
}

#define t_set_ipm(intpri)    x_set_ipm(intpri)
#define i_set_ipm(intpri)    x_set_ipm(intpri)

/*
 * �i���f����́j�����ݗD��x�}�X�N�̎Q��
 *
 */
Inline PRI
x_get_ipm(void)
{
	return EXT_IPM(iipm);
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
		ief_systick &= ~0x01;
	}else {
		tmp = intno - 16;
		sil_wrw_mem((void *)(NVIC_CLRENA0), (1 << (tmp & 0x1f)));
		ief &= ~(1 << (tmp & 0x1f));
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
		ief_systick |= 0x01;
		if ((iipm_enable_systic_tbl[iipm] & ief_systick) ==  0x01) {
			tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
			tmp |= SYSTIC_TICINT;;
			sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
		}
	}else {
		tmp = intno - 16;
		ief |= (1 << (tmp & 0x1f));
		if ((iipm_enable_irq_tbl[iipm] & (1 << (tmp & 0x1f))) != 0) {
			sil_wrw_mem((void *)(NVIC_SETENA0), (1 << (tmp & 0x1f)));
		}
	}

	return(true);
}

#define t_enable_int(intno) x_enable_int(intno)
#define i_enable_int(intno) x_enable_int(intno)

/*
 *  PendSVC�n���h���icore_support.S�j
 */
extern void pendsvc_handler(void);

#endif /* TOPPERS_MACRO_ONLY */
#endif /* TOPPERS_CORE_INTMODEL_V6M_H */
