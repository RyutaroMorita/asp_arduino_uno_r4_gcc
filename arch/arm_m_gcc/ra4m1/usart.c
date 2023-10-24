/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2007,2011,2013,2015 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 */

/*
 *  シリアルドライバ（RA4M1 USART用）
 */

#include <kernel.h>
#include <sil.h>
#include "usart.h"
#include "target_syssvc.h"
#include "hal_data.h"

/*
 * レジスタ設定値
 */
#define PORT2SIOPID(x)	((x) + 1)
#define INDEX_PORT(x)	((x) - 1)
#define GET_SIOPCB(x)	(&siopcb_table[INDEX_PORT(x)])

/*
 * SCIレジスタ定義
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
  *  シリアルポートの管理ブロック
  */
struct sio_port_control_block {
	ID port;
	const SIOPINIB *p_siopinib;	/* シリアルI/Oポート初期化ブロック */
	intptr_t exinf;
};

#define IELSR4RX	30
#define IELSR4TX	31

/*
 *  シリアルI/Oポート管理ブロックエリア
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
 *  ターゲットのシリアル初期化
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

	/* 既にオープンされている場合は初期化しない */
	if (siopid == SIO_PORTID) {
		/* ステータスのクリア */
		if (sil_reb_mem((void*)SCI_SSR(p_siopinib->reg)))
			sil_wrb_mem((void*)SCI_SSR(p_siopinib->reg), 0x00U);
		return;
	}

	/* ボーレートの計算 */
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
		while (1);	// パラメーターエラー

    /* UART停止 */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), 0x00U);

	/*  8データ,1ストップビット,no parity */
	sil_wrb_mem((void*)SCI_SMR(p_siopinib->reg), (uint8_t)cks);
	sil_wrb_mem((void*)SCI_SEMR(p_siopinib->reg), semr);

	/* ボーレートを設定 */
	sil_wrb_mem((void*)SCI_BRR(p_siopinib->reg), (uint8_t)brr);

	/* ステータスのクリア */
	if (sil_reb_mem((void*)SCI_SSR(p_siopinib->reg)))
		sil_wrb_mem((void*)SCI_SSR(p_siopinib->reg), 0x00U);

	/* UART開始 */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), (SCI_SCR_RE | SCI_SCR_TE));

	/*
	 *  イベント（割り込み）リンク設定
	 */
	sil_wrw_mem((void*)ICU_IELSR(IELSR4RX), p_siopinib->iels_rx);
	sil_wrw_mem((void*)ICU_IELSR(IELSR4TX), p_siopinib->iels_tei);
}

/*
 *  ターゲットのシリアル終了
 */
static void
usart_term(ID siopid)
{
	SIOPCB *p_siopcb;
	const SIOPINIB	*p_siopinib;

	p_siopcb = GET_SIOPCB(siopid);
	p_siopinib = p_siopcb->p_siopinib;

    /* UART停止 */
	sil_wrb_mem((void*)SCI_SCR(p_siopinib->reg), 0x00U);
}

/*
 *  SIO初期化
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
 *  シリアルオープン
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

	/* 既にオープンされている場合は初期化しない */
	//if (siopid != TNUM_PORT)
		usart_init(siopid);

	return p_siopcb;
}

/*
 *  シリアルクローズ
 */
void
sio_cls_por(SIOPCB *p_siopcb)
{
	usart_term(PORT2SIOPID(p_siopcb->port));
}

/*
 *  受信割込みサービスルーチン
 */
void
sio_isr_rx(intptr_t exinf)
{
	SIOPCB *p_siopcb;

	p_siopcb = GET_SIOPCB(exinf);

	/*
	 *  RX割り込み検出はレベル検出のため
	 *  ICUのIELSRレジスタのIRビットをクリアする必要がある．
	 */
	sil_andw((void*)ICU_IELSR(IELSR4RX), ~ICU_IELSR_IR);

	if (sio_getready(p_siopcb)) {
		sio_irdy_rcv(p_siopcb->exinf);
	}
}

/*
 *  送信割込みサービスルーチン
 */
void
sio_isr_tx(intptr_t exinf)
{
	SIOPCB *p_siopcb;

	p_siopcb = GET_SIOPCB(exinf);

	/*
	 *  TEI割り込み検出はレベル検出のため
	 *  ICUのIELSRレジスタのIRビットをクリアする必要がある．
	 */
	sil_andw((void*)ICU_IELSR(IELSR4TX), ~ICU_IELSR_IR);

	if (sio_putready(p_siopcb)) {
		sio_irdy_snd(p_siopcb->exinf);
	}
}

/*
 *  1文字送信
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
 *  1文字受信
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
 *  コールバックの許可
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
 *  コールバックの禁止
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
 *  1文字出力（ポーリングでの出力）
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
