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
 *  ターゲット依存モジュール（Arduino UNO R4用）
 */
#include "kernel_impl.h"
#include <sil.h>
#include "target_serial.h"
#include "target_syssvc.h"
#include "hal_data.h"

/*
 * 　PRCRレジスタに書き込むためのキーコード
 */
#define BSP_PRV_PRCR_KEY                              (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK                      ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_LOCK                             ((BSP_PRV_PRCR_KEY) | 0x0U)

/*
 * 　bsp_lock_cfg.hのHOCO 周波数設定とbsp_cfg.hのOFS1設定のOR
 */
#define BSP_ROM_REG_OFS1_SETTING                                             \
    (((uint32_t) BSP_CFG_ROM_REG_OFS1 & BSP_FEATURE_BSP_OFS1_HOCOFRQ_MASK) | \
     ((uint32_t) BSP_CFG_HOCO_FREQUENCY << BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET))

/*
 * 　MPUの設定に基づいたSECMPUACレジスタの構築します
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
 *  エラー時の処理
 */
extern void Error_Handler(void);

/*
 *  hal_entry.cのR_BSP_WarmStart()参照
 */
extern void R_BSP_WarmStart(bsp_warm_start_event_t event);

/*
 *  システムクロック
 */
uint32_t SystemCoreClock;

/*
 *  バーナ出力用のUARTの初期化
 */
static void usart_early_init(void);

/*
 * 　ROMレジスタの定義
 *
 *  BSP_SECTION_ROM_REGISTERSで示すセクション（通常は「.rom_registers」）に配置される．
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
 * 　IDコードの定義
 *
 *  BSP_SECTION_ID_CODEで示すセクション（通常は「.id_code」）に配置される．
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
 * 　ヒープ領域の設定
 *
 *  BSP_SECTION_HEAPで示すセクション（通常は「.heap」）に配置される．
 * 　標準ライブラリが使用するヒープ領域は，通常，現在のスタックポインタよりも若いアドレスに配置されなければならない．
 *  TOPPERS/ASPを使用する場合，スタックポインタは常にBSS領域を移動する．
 *  そのため「.heap」セクションは,通常とは逆に「.bss」セクションの前に配置されるように「arduino_uno_r4.ld」に
 *  記述している.
 */
#if (BSP_CFG_HEAP_BYTES > 0)

BSP_DONT_REMOVE static uint8_t g_heap[BSP_CFG_HEAP_BYTES] BSP_ALIGN_VARIABLE(BSP_STACK_ALIGNMENT) \
    BSP_PLACE_IN_SECTION(BSP_SECTION_HEAP);
#endif

/*
 *  起動時のハードウェア初期化処理
 */
void
hardware_init_hook(void) {
	/*
	 *  -fdata-sectionsを使用するとistkが削除され，
	 *  cfgのパス3のチェックがエラーとなるため，
	 *  削除されないようにする
	 */
	SystemCoreClock = (uint32_t)istk;
}

/*
 *  ターゲット依存部 初期化処理
 */
void
target_initialize(void)
{
#if BSP_FEATURE_BSP_RESET_TRNG
    volatile uint8_t read_port = 0U;
    FSP_PARAMETER_NOT_USED(read_port);	/* コンパイラの'unused'警告を防ぐ */
#endif

#if BSP_FEATURE_BSP_VBATT_HAS_VBTCR1_BPWSWSTP

    /*
     *　 VBTCR1レジスタのアンロック
     */
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_PRC1_UNLOCK;

    /*
     *  VBTCR1.BPWSWSTPを持つMCUでは，リセット後にVBTCR1.BPWSWSTPを設定する必要がある．
     *  RA4M1マニュアルR01UM0007EU0110の「11.2.1 VBATTコントロールレジスタ1(VBTCR1)」および
     *  「図11.2 VBTCR1.BPWSWSTPビットの設定フロー」を参照のこと．
     *  VBTSR.VBTRVLDが設定されるまでLOCOCR、LOCOUTCR、SOSCCR，およびSOMCRにアクセスできないため，
     *  これはbsp_lock_init()の前に行う必要がある．
     */
    R_SYSTEM->VBTCR1 = 1U;
    FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->VBTSR_b.VBTRVLD, 1U);

    /*
     * 　VBTCR1レジスタのロック
     */
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_LOCK;
#endif

    /*
     *  クロック初期化前に必要な処理
     */
    R_BSP_WarmStart(BSP_WARM_START_RESET);

    /*
     *　 システムクロックの設定
     */
    bsp_clock_init();

#if BSP_FEATURE_BSP_RESET_TRNG

    /*
     *  望ましくない電流の引き込みを防ぐために，このMCUではクロックの初期化後に
     *  TRNG回路をリセットする必要がある．
     */

    /*
     *  低消費電力モード時のレジスタ保護解除（RA2A1ユーザーズマニュアル（R01UH0888JJ0100）
     *  図11.13「未使用回路の初期設定フロー例」による）
     */
    R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT);

    /*
     *  TRNG機能の有効化（ストップ機能の無効化）
     */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_START(FSP_IP_TRNG, 0); // for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_START(FSP_IP_SCE, 0);  // for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /*
     *  最低3PCLKBサイクル待機
     */
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;
    read_port = R_PFS->PORT[0].PIN[0].PmnPFS_b.PODR;

    /*
     *  TRNG機能の無効化
     */
 #if BSP_FEATURE_BSP_HAS_SCE_ON_RA2
    R_BSP_MODULE_STOP(FSP_IP_TRNG, 0); // for RA2 series.
 #elif BSP_FEATURE_BSP_HAS_SCE5
    R_BSP_MODULE_STOP(FSP_IP_SCE, 0);  // for RA4 series.
 #else
  #error "BSP_FEATURE_BSP_RESET_TRNG is defined but not handled."
 #endif

    /*
     *  低電力モード用レジスタ保護の再適用（RA2A1ユーザーズマニュアル（R01UH0888JJ0100）
     *  図11.13「未使用回路の初期設定フロー例」による）
     */
    R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT);
#endif

    /*
     *  クロック初期化後に必要な処理
     */
    R_BSP_WarmStart(BSP_WARM_START_POST_CLOCK);

    /*
     *  MSP監視を無効化
     */
    R_MPU_SPMON->SP[0].CTL = 0;

    /*
     *  SystemCoreClock変数の初期化
     */
    SystemCoreClockUpdate();

#if !BSP_CFG_PFS_PROTECT
    R_PMISC->PWPR = 0;                              ///< Clear BOWI bit - writing to PFSWE bit enabled
    R_PMISC->PWPR = 1U << BSP_IO_PWPR_PFSWE_OFFSET; ///< Set PFSWE bit - writing to PFS register enabled
#endif

    /*
     *  Cランタイム後に必要な処理
     */
    R_BSP_WarmStart(BSP_WARM_START_POST_C);

	/*
	 *  target_fput_logが使えるようにUARTを初期化
	 */
#if (SIO_PORTID == 2)
	/*
	 * 　SCI1モジュールストップ設定
	 */
	R_MSTP->MSTPCRB_b.MSTPB30 = 0;	/* モジュールストップの解除 */
#endif

	/*
	 *  バーナー出力用のシリアル初期化
	 */
	usart_early_init();
}

/*
 * ターゲット依存部 終了処理
 */
void
target_exit(void)
{
	/*
	 *　　チップ依存部の終了処理
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

	/* ボーレートの計算 */
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
		target_exit();	// パラメーターエラー

#if (SIO_PORTID == 2)
	/*
	 *  SCI1の初期化
	 */
	/* SCI1停止 */
	R_SCI1->SCR = 0;
	/*  8データ,1ストップビット,no parity */
	R_SCI1->SMR = (uint8_t)cks;
	R_SCI1->SEMR = semr;
	/* ボーレートを設定 */
	R_SCI1->BRR = (uint8_t)brr;
	/* ステータスのクリア */
	if (R_SCI1->SSR)
		R_SCI1->SSR = 0;
	/* UART開始 */
	R_SCI1->SCR_b.RE = 1;
	R_SCI1->SCR_b.TE = 1;

	/*
	 *  イベント（割り込み）リンク設定
	 */
	R_ICU->IELSR_b[30].IELS = 0x9E;
	R_ICU->IELSR_b[31].IELS = 0xA0;
#endif
}

/*
 * 　システムログの低レベル出力のための文字出力
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
 *　 初期化時のエラー発生時の処理
 */
void
Error_Handler(void){
	volatile int loop;
	while(1){
		for(loop = 0; loop < 0x100000; loop++);
	}
}
