/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2007,2011,2013,2015 by Embedded and Real-Time Systems Laboratory
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
 *  �V���A���h���C�o�iRA4M1 USART�p�j
 */

#include <kernel.h>
#include <sil.h>
#include "usart.h"
#include "target_syssvc.h"
#include "hal_data.h"

/*
 * ���W�X�^�ݒ�l
 */
#define PORT2SIOPID(x)	((x) + 1)
#define INDEX_PORT(x)	((x) - 1)
#define GET_SIOPCB(x)	(&siopcb_table[INDEX_PORT(x)])

/*
 * SCI���W�X�^��`
 */
#define SCI_SMR(x)		(x + 0x00)
#define SCI_BRR(x)		(x + 0x01)
#define SCI_SCR(x)		(x + 0x02)
#define SCI_TDR(x)		(x + 0x03)
#define SCI_SSR(x)		(x + 0x04)
#define SCI_RDR(x)		(x + 0x05)
#define SCI_SCMR(x)		(x + 0x06)
#define SCI_SEMR(x)		(x + 0x07)
#define SCI_SNFR(x)		(x + 0x08)
#define SCI_SIMR1(x)	(x + 0x09)
#define SCI_SIMR2(x)	(x + 0x0A)
#define SCI_SIMR3(x)	(x + 0x0B)
#define SCI_SISR(x)		(x + 0x0C)
#define SCI_SPMR(x)		(x + 0x0D)
#define SCI_TDRHL(x)	(x + 0x0E)
//#define SCI_FTDRHL(x)	(x + 0x0E)
//#define SCI_FTDRH(x)	(x + 0x0E)
//#define SCI_FTDRL(x)	(x + 0x0F)
#define SCI_RDRHL(x)	(x + 0x10)
//#define SCI_FRDRHL(x)	(x + 0x10)
//#define SCI_FRDRH(x)	(x + 0x10)
//#define SCI_FRDRL(x)	(x + 0x11)
#define SCI_MDDR(x)		(x + 0x12)
#define SCI_DCCR(x)		(x + 0x13)
//#define SCI_FCR(x)		(x + 0x14)
//#define SCI_FDR(x)		(x + 0x16)
//#define SCI_LSR(x)		(x + 0x18)
#define SCI_CDR(x)		(x + 0x1A)
#define SCI_SPTR(x)		(x + 0x1C)

#define SCI_SCR_TEIE	(1 <<  2)
#define SCI_SCR_RE		(1 <<  4)
#define SCI_SCR_TE		(1 <<  5)
#define SCI_SCR_RIE		(1 <<  6)
#define SCI_SCR_TIE		(1 <<  7)

#define SCI_SSR_TEND	(1 <<  2)
#define SCI_SSR_RDRF	(1 <<  6)
#define SCI_SSR_TDRE	(1 <<  7)

#define ICU_IELSR(n)	(R_ICU_BASE + 0x300UL + (n * 4))
#define ICU_IELSR_IR	(1 << 16)

extern uint32_t SystemCoreClock;

typedef struct{
	uint32_t reg;
	uint32_t iels_rx;
	uint32_t iels_tei;
} SIOPINIB;

 /*
  *  �V���A���|�[�g�̊Ǘ��u���b�N
  */
struct sio_port_control_block {
	ID port;
	const SIOPINIB *p_siopinib;	/* �V���A��I/O�|�[�g�������u���b�N */
	intptr_t exinf;
};

#define IELSR4RX	30
#define IELSR4TX	31

/*
 *  �V���A��I/O�|�[�g�Ǘ��u���b�N�G���A
 */
SIOPCB siopcb_table[TNUM_PORT];

static const SIOPINIB siopinib_table[TNUM_PORT] = {
	{	// SCI0
		R_SCI0_BASE,
		0x98,
		0x9A
	},
#if (TNUM_PORT >= 2)
	{	// SCI1
		R_SCI1_BASE,
		0x9E,
		0xA0
	},
#endif
#if (TNUM_PORT >= 3)
	{	// SCI2
		R_SCI2_BASE,
		0xA3,
		0xA5
	},
#endif
#if (TNUM_PORT >= 4)
	{	// SCI9
		R_SCI9_BASE,
		0xA8,
		0xAA
	},
#endif
};

const SEMR_MODE_TABLE semr_mode[] = {
	{	(12 / 2), (R_SCI0_SEMR_ABCSE_Msk)},
	{	(16 / 2), (R_SCI0_SEMR_BGDM_Msk | R_SCI0_SEMR_ABCS_Msk)	},
	{	(32 / 2), (R_SCI0_SEMR_BGDM_Msk),						},
	{	(64 / 2), 0,										},
};

Inline bool_t
sio_putready(SIOPCB* siopcb)
{
	return((sil_reb_mem((void*)SCI_SSR(siopcb->p_siopinib->reg)) & SCI_SSR_TEND) != 0);
}

Inline bool_t
sio_getready(SIOPCB* siopcb)
{
	return((sil_reb_mem((void*)SCI_SSR(siopcb->p_siopinib->reg)) & SCI_SSR_RDRF) != 0);
}

/*
 *  �^�[�Q�b�g�̃V���A��������
 */
void
usart_init(ID siopid)
{
	SIOPCB *p_siopcb;
	const SIOPINIB	*p_siopinib;
	uint32_t cks, i, j;
	uint32_t tmp;
	uint32_t val;
	uint32_t brr = 0;
	uint8_t semr = 0;

	p_siopcb = GET_SIOPCB(siopid);
	p_siopinib = p_siopcb->p_siopinib;

	/* ���ɃI�[�v������Ă���ꍇ�͏��������Ȃ� */
	if (siopid == SIO_PORTID) {
		/* �X�e�[�^�X�̃N���A */
		if (sil_reb_mem((void*)SCI_SSR(p_siopinib->reg)))
			sil_wrb_mem((void*)SCI_SSR(p_siopinib->reg), 0x00U);
		return;
	}

	/* �{�[���[�g�̌v�Z */
	for (i = 0; i < NUM_SEMR_MODE; i++) {
		tmp = semr_mode[i].tmp;
		for (cks = 0; cks < 3; cks++) {
			val = tmp;
			for (j = 0; j < cks; j++)
				val *= 4;
			brr = SystemCoreClock / (val * BPS_SETTING) - 1;
			if (brr < 0x100)
				break;
		}
		if (brr < 0x100) {
			semr = semr_mode[i].semr;
			break;
		}
	}
	if (brr >= 0x100)
		while (1);	// �p�����[�^�[�G���[

    /* UART��~ */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), 0x00U);

	/*  8�f�[�^,1�X�g�b�v�r�b�g,no parity */
	sil_wrb_mem((void*)SCI_SMR(p_siopinib->reg), (uint8_t)cks);
	sil_wrb_mem((void*)SCI_SEMR(p_siopinib->reg), semr);

	/* �{�[���[�g��ݒ� */
	sil_wrb_mem((void*)SCI_BRR(p_siopinib->reg), (uint8_t)brr);

	/* �X�e�[�^�X�̃N���A */
	if (sil_reb_mem((void*)SCI_SSR(p_siopinib->reg)))
		sil_wrb_mem((void*)SCI_SSR(p_siopinib->reg), 0x00U);

	/* UART�J�n */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), (SCI_SCR_RE | SCI_SCR_TE));

	/*
	 *  �C�x���g�i���荞�݁j�����N�ݒ�
	 */
	sil_wrw_mem((void*)ICU_IELSR(IELSR4RX), p_siopinib->iels_rx);
	sil_wrw_mem((void*)ICU_IELSR(IELSR4TX), p_siopinib->iels_tei);
}

/*
 *  �^�[�Q�b�g�̃V���A���I��
 */
static void
usart_term(ID siopid)
{
	SIOPCB *p_siopcb;
	const SIOPINIB	*p_siopinib;

	p_siopcb = GET_SIOPCB(siopid);
	p_siopinib = p_siopcb->p_siopinib;

    /* UART��~ */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), 0x00U);
}

/*
 *  SIO������
 */
void
sio_initialize(intptr_t exinf)
{
	int i;

	for (i = 0; i < TNUM_PORT; i++) {
		siopcb_table[i].port = i;
		siopcb_table[i].p_siopinib = &siopinib_table[i];
		siopcb_table[i].exinf = 0;
	}
}

/*
 *  �V���A���I�[�v��
 */
SIOPCB
*sio_opn_por(ID siopid, intptr_t exinf)
{
	SIOPCB *p_siopcb;

	if (siopid > TNUM_PORT) {
		return NULL;
	}

	p_siopcb = GET_SIOPCB(siopid);
	p_siopcb->exinf = exinf;

	/* ���ɃI�[�v������Ă���ꍇ�͏��������Ȃ� */
	//if (siopid != TNUM_PORT)
		usart_init(siopid);

	return p_siopcb;
}

/*
 *  �V���A���N���[�Y
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	usart_term(PORT2SIOPID(p_siopcb->port));
}

/*
 *  ��M�����݃T�[�r�X���[�`��
 */
void
sio_isr_rx(intptr_t exinf)
{
	SIOPCB *p_siopcb;

	p_siopcb = GET_SIOPCB(exinf);

	/*
	 *  RX���荞�݌��o�̓��x�����o�̂���
	 *  ICU��IELSR���W�X�^��IR�r�b�g���N���A����K�v������D
	 */
	sil_andw((void*)ICU_IELSR(IELSR4RX), ~ICU_IELSR_IR);

	if (sio_getready(p_siopcb)) {
		sio_irdy_rcv(p_siopcb->exinf);
	}
}

/*
 *  ���M�����݃T�[�r�X���[�`��
 */
void
sio_isr_tx(intptr_t exinf)
{
	SIOPCB *p_siopcb;

	p_siopcb = GET_SIOPCB(exinf);

	/*
	 *  TEI���荞�݌��o�̓��x�����o�̂���
	 *  ICU��IELSR���W�X�^��IR�r�b�g���N���A����K�v������D
	 */
	sil_andw((void*)ICU_IELSR(IELSR4TX), ~ICU_IELSR_IR);

	if (sio_putready(p_siopcb)) {
		sio_irdy_snd(p_siopcb->exinf);
	}
}

/*
 *  1�������M
 */
bool_t
sio_snd_chr(SIOPCB *siopcb, char c)
{
	const SIOPINIB	*p_siopinib = siopcb->p_siopinib;

	if (sio_putready(siopcb)) {
		sil_wrb_mem((void*)SCI_TDR(p_siopinib->reg), c);
		return true;
	}
	return false;
}

/*
 *  1������M
 */
int_t
sio_rcv_chr(SIOPCB *siopcb)
{
	const SIOPINIB	*p_siopinib = siopcb->p_siopinib;

	int_t c = -1;
	if (sio_getready(siopcb)) {
		c = sil_reb_mem((void*)SCI_RDR(p_siopinib->reg)) & 0xFF;
	}
	return c;
}

/*
 *  �R�[���o�b�N�̋���
 */
void
sio_ena_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	const SIOPINIB	*p_siopinib = siopcb->p_siopinib;

	switch (cbrtn) {
	case SIO_RDY_SND:
		sil_orb((void*)SCI_SCR(p_siopinib->reg), SCI_SCR_TEIE);
		break;
	case SIO_RDY_RCV:
		sil_orb((void*)SCI_SCR(p_siopinib->reg), SCI_SCR_RIE);
		break;
	default:
		break;
	}
}

/*
 *  �R�[���o�b�N�̋֎~
 */
void
sio_dis_cbr(SIOPCB *siopcb, uint_t cbrtn)
{
	const SIOPINIB	*p_siopinib = siopcb->p_siopinib;

	switch (cbrtn) {
	case SIO_RDY_SND:
		sil_andb((void*)SCI_SCR(p_siopinib->reg), ~SCI_SCR_TEIE);
		break;
	case SIO_RDY_RCV:
		sil_andb((void*)SCI_SCR(p_siopinib->reg), ~SCI_SCR_RIE);
		break;
	default:
		break;
	}
}

/*
 *  1�����o�́i�|�[�����O�ł̏o�́j
 */
void
sio_pol_snd_chr(char c, ID siopid)
{
	SIOPCB *p_siopcb;
	const SIOPINIB	*p_siopinib;

	p_siopcb = GET_SIOPCB(siopid);
	p_siopinib = p_siopcb->p_siopinib;

	while ((sil_reb_mem((void*)SCI_SSR(p_siopinib->reg)) & SCI_SSR_TEND) == 0);
	sil_wrb_mem((void*)SCI_TDR(p_siopinib->reg), c);
}
