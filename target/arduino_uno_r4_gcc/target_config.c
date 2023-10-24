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

/*
 *  �^�[�Q�b�g�ˑ����W���[���iArduino UNO R4�p�j
 */
#include "kernel_impl.h"
#include <sil.h>
#include "target_serial.h"
#include "target_syssvc.h"
#include "hal_data.h"

/*
 * �@PRCR���W�X�^�ɏ������ނ��߂̃L�[�R�[�h
 */
#define BSP_PRV_PRCR_KEY                              (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK                      ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_LOCK                             ((BSP_PRV_PRCR_KEY) | 0x0U)

/*
 * �@bsp_lock_cfg.h��HOCO ���g���ݒ��bsp_cfg.h��OFS1�ݒ��OR
 */
#define BSP_ROM_REG_OFS1_SETTING                                             \
    (((uint32_t) BSP_CFG_ROM_REG_OFS1 & BSP_FEATURE_BSP_OFS1_HOCOFRQ_MASK) | \
     ((uint32_t) BSP_CFG_HOCO_FREQUENCY << BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET))

/*
 * �@MPU�̐ݒ�Ɋ�Â���SECMPUAC���W�X�^�̍\�z���܂�
 */
#define BSP_ROM_REG_MPU_CONTROL_SETTING                     \
    ((0xFFFFFCF0U) |                                        \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_ENABLE << 8) |     \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_ENABLE << 9) |     \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_ENABLE) |      \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_ENABLE << 1) | \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_ENABLE << 2) | \
     ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_ENABLE << 3))

/*
 *  �G���[���̏���
 */
extern void Error_Handler(void);

/*
 *  hal_entry.c��R_BSP_WarmStart()�Q��
 */
extern void R_BSP_WarmStart(bsp_warm_start_event_t event);

/*
 *  �V�X�e���N���b�N
 */
uint32_t SystemCoreClock;

/*
 *  �o�[�i�o�͗p��UART�̏�����
 */
static void usart_early_init(void);

/*
 * �@ROM���W�X�^�̒�`
 *
 *  BSP_SECTION_ROM_REGISTERS�Ŏ����Z�N�V�����i�ʏ�́u.rom_registers�v�j�ɔz�u�����D
 */
BSP_DONT_REMOVE static const uint32_t g_bsp_rom_registers[] BSP_PLACE_IN_SECTION (BSP_SECTION_ROM_REGISTERS) =
{
    (uint32_t) BSP_CFG_ROM_REG_OFS0,
    (uint32_t) BSP_ROM_REG_OFS1_SETTING,
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC0_END | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_PC1_END | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_START & BSP_FEATURE_BSP_MPU_REGION0_MASK & 0xFFFFFFFCU),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION0_END & BSP_FEATURE_BSP_MPU_REGION0_MASK) | 0x00000003U),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_START & 0xFFFFFFFCU),
    ((uint32_t) BSP_CFG_ROM_REG_MPU_REGION1_END | 0x00000003U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_START & 0x407FFFFCU) | 0x40000000U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION2_END & 0x407FFFFCU) | 0x40000003U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_START & 0x407FFFFCU) | 0x40000000U),
    (((uint32_t) BSP_CFG_ROM_REG_MPU_REGION3_END & 0x407FFFFCU) | 0x40000003U),
    (uint32_t) BSP_ROM_REG_MPU_CONTROL_SETTING
};

/*
 * �@ID�R�[�h�̒�`
 *
 *  BSP_SECTION_ID_CODE�Ŏ����Z�N�V�����i�ʏ�́u.id_code�v�j�ɔz�u�����D
 */
BSP_DONT_REMOVE static const uint32_t g_bsp_id_codes[] BSP_PLACE_IN_SECTION (BSP_SECTION_ID_CODE) =
{
    BSP_CFG_ID_CODE_LONG_1,
#if BSP_FEATURE_BSP_OSIS_PADDING
    0xFFFFFFFFU,
#endif
    BSP_CFG_ID_CODE_LONG_2,
#if BSP_FEATURE_BSP_OSIS_PADDING
    0xFFFFFFFFU,
#endif
    BSP_CFG_ID_CODE_LONG_3,
#if BSP_FEATURE_BSP_OSIS_PADDING
    0xFFFFFFFFU,
#endif
    BSP_CFG_ID_CODE_LONG_4
};

/*
 * �@�q�[�v�̈�̐ݒ�
 *
 *  BSP_SECTION_HEAP�Ŏ����Z�N�V�����i�ʏ�́u.heap�v�j�ɔz�u�����D
 * �@�W�����C�u�������g�p����q�[�v�̈�́C�ʏ�C���݂̃X�^�b�N�|�C���^�����Ⴂ�A�h���X�ɔz�u����Ȃ���΂Ȃ�Ȃ��D
 *  TOPPERS/ASP���g�p����ꍇ�C�X�^�b�N�|�C���^�͏��BSS�̈���ړ�����D
 *  ���̂��߁u.heap�v�Z�N�V������,�ʏ�Ƃ͋t�Ɂu.bss�v�Z�N�V�����̑O�ɔz�u�����悤�Ɂuarduino_uno_r4.ld�v��
 *  �L�q���Ă���.
 */
#if (BSP_CFG_HEAP_BYTES > 0)

BSP_DONT_REMOVE static uint8_t g_heap[BSP_CFG_HEAP_BYTES] BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT) \
    BSP_PLACE_IN_SECTION(BSP_SECTION_HEAP);
#endif

/*
 *  �N�����̃n�[�h�E�F�A����������
 */
void
hardware_init_hook(void) {
	/*
	 *  -fdata-sections���g�p�����istk���폜����C
	 *  cfg�̃p�X3�̃`�F�b�N���G���[�ƂȂ邽�߁C
	 *  �폜����Ȃ��悤�ɂ���
	 */
	SystemCoreClock = (uint32_t)istk;
}

/*
 *  �^�[�Q�b�g�ˑ��� ����������
 */
void
target_initialize(void)
{
#if BSP_FEATURE_BSP_RESET_TRNG
    volatile uint8_t read_port = 0U;
    FSP_PARAMETER_NOT_USED(read_port);	/* �R���p�C����'unused'�x����h�� */
#endif

#if BSP_FEATURE_BSP_VBATT_HAS_VBTCR1_BPWSWSTP

    /*
     *�@ VBTCR1���W�X�^�̃A�����b�N
     */
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_PRC1_UNLOCK;

    /*
     *  VBTCR1.BPWSWSTP������MCU�ł́C���Z�b�g���VBTCR1.BPWSWSTP��ݒ肷��K�v������D
     *  RA4M1�}�j���A��R01UM0007EU0110�́u11.2.1 VBATT�R���g���[�����W�X�^1(VBTCR1)�v�����
     *  �u�}11.2 VBTCR1.BPWSWSTP�r�b�g�̐ݒ�t���[�v���Q�Ƃ̂��ƁD
     *  VBTSR.VBTRVLD���ݒ肳���܂�LOCOCR�ALOCOUTCR�ASOSCCR�C�����SOMCR�ɃA�N�Z�X�ł��Ȃ����߁C
     *  �����bsp_lock_init()�̑O�ɍs���K�v������D
     */
    R_SYSTEM->VBTCR1 = 1U;
    FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->VBTSR_b.VBTRVLD, 1U);

    /*
     * �@VBTCR1���W�X�^�̃��b�N
     */
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_LOCK;
#endif

    /*
     *  �N���b�N�������O�ɕK�v�ȏ���
     */
    R_BSP_WarmStart(BSP_WARM_START_RESET);

    /*
     *�@ �V�X�e���N���b�N�̐ݒ�
     */
    bsp_clock_init();

#if BSP_FEATURE_BSP_RESET_TRNG

    /*
     *  �]�܂����Ȃ��d���̈������݂�h�����߂ɁC����MCU�ł̓N���b�N�̏��������
     *  TRNG��H�����Z�b�g����K�v������D
     */

    /*
     *  �����d�̓��[�h���̃��W�X�^�ی�����iRA2A1���[�U�[�Y�}�j���A���iR01UH0888JJ0100�j
     *  �}11.13�u���g�p��H�̏����ݒ�t���[��v�ɂ��j
     */
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);

    /*
     *  TRNG�@�\�̗L�����i�X�g�b�v�@�\�̖������j
     */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_START(FSP_IP_TRNG, 0); // for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_START(FSP_IP_SCE, 0);  // for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /*
     *  �Œ�3PCLKB�T�C�N���ҋ@
     */
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;

    /*
     *  TRNG�@�\�̖�����
     */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_STOP(FSP_IP_TRNG, 0); // for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_STOP(FSP_IP_SCE, 0);  // for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /*
     *  ��d�̓��[�h�p���W�X�^�ی�̍ēK�p�iRA2A1���[�U�[�Y�}�j���A���iR01UH0888JJ0100�j
     *  �}11.13�u���g�p��H�̏����ݒ�t���[��v�ɂ��j
     */
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);
#endif

    /*
     *  �N���b�N��������ɕK�v�ȏ���
     */
    R_BSP_WarmStart(BSP_WARM_START_POST_CLOCK);

    /*
     *  MSP�Ď��𖳌���
     */
    R_MPU_SPMON->SP[0].CTL = 0;

    /*
     *  SystemCoreClock�ϐ��̏�����
     */
    SystemCoreClockUpdate();

#if !BSP_CFG_PFS_PROTECT
    R_PMISC->PWPR = 0;                              ///< Clear BOWI bit - writing to PFSWE bit enabled
    R_PMISC->PWPR = 1U << BSP_IO_PWPR_PFSWE_OFFSET; ///< Set PFSWE bit - writing to PFS register enabled
#endif

    /*
     *  C�����^�C����ɕK�v�ȏ���
     */
    R_BSP_WarmStart(BSP_WARM_START_POST_C);

	/*
	 *  target_fput_log���g����悤��UART��������
	 */
#if (SIO_PORTID == 2)
	/*
	 * �@SCI1���W���[���X�g�b�v�ݒ�
	 */
	R_MSTP->MSTPCRB_b.MSTPB30 = 0;	/* ���W���[���X�g�b�v�̉��� */
#endif

	/*
	 *  �o�[�i�[�o�͗p�̃V���A��������
	 */
	usart_early_init();
}

/*
 * �^�[�Q�b�g�ˑ��� �I������
 */
void
target_exit(void)
{
	/*
	 *�@�@�`�b�v�ˑ����̏I������
	 */
	core_terminate();
	while(1);
}

extern SEMR_MODE_TABLE semr_mode[];

static void usart_early_init()
{
	uint32_t cks, i, j;
	uint32_t tmp;
	uint32_t val;
	uint32_t brr = 0;
	uint8_t semr = 0;

	/* �{�[���[�g�̌v�Z */
	for (i = 0; i < NUM_SEMR_MODE; i++) {
		tmp = semr_mode[i].tmp;
		for (cks = 0; cks < 3; cks++) {
			val = tmp;
			for (j = 0; j < cks; j++)
				val *= 4;
			brr = SystemCoreClock / (val * BPS_SETTING) - 1;
			if (brr < 0x100UL)
				break;
		}
		if (brr < 0x100UL) {
			semr = semr_mode[i].semr;
			break;
		}
	}
	if (brr >= 0x100UL)
		target_exit();	// �p�����[�^�[�G���[

#if (SIO_PORTID == 2)
	/*
	 *  SCI1�̏�����
	 */
	/* SCI1��~ */
	R_SCI1->SCR = 0;
	/*  8�f�[�^,1�X�g�b�v�r�b�g,no parity */
	R_SCI1->SMR = (uint8_t)cks;
	R_SCI1->SEMR = semr;
	/* �{�[���[�g��ݒ� */
	R_SCI1->BRR = (uint8_t)brr;
	/* �X�e�[�^�X�̃N���A */
	if (R_SCI1->SSR)
		R_SCI1->SSR = 0;
	/* UART�J�n */
	R_SCI1->SCR_b.RE = 1;
	R_SCI1->SCR_b.TE = 1;

	/*
	 *  �C�x���g�i���荞�݁j�����N�ݒ�
	 */
	R_ICU->IELSR_b[30].IELS = 0x9E;
	R_ICU->IELSR_b[31].IELS = 0xA0;
#endif
}

/*
 * �@�V�X�e�����O�̒჌�x���o�͂̂��߂̕����o��
 */
void
target_fput_log(char c)
{
#ifndef TOPPERS_OMIT_SYSLOG
 #if (SIO_PORTID == 2)
	if (c == '\n') {
		while (R_SCI1->SSR_b.TEND == 0);
		R_SCI1->TDR = '\r';
	}
	while (R_SCI1->SSR_b.TEND == 0);
	R_SCI1->TDR = c;
 #endif
#endif
}

/*
 *�@ ���������̃G���[�������̏���
 */
void
Error_Handler(void){
	volatile int loop;
	while(1){
		for(loop = 0; loop < 0x100000; loop++);
	}
}
