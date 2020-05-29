/***************************************************************************

  ym2610.h

  Header file for software emulation for YAMAHA YM-2610 sound generator

***************************************************************************/

#ifndef _YM2610_H_
#define _YM2610_H_


/* --- speedup optimize --- */
/* busy flag enulation , The definition of FM_GET_TIME_NOW() is necessary. */
#define FM_BUSY_FLAG_SUPPORT 1

/*------------------------------------------------------------------------*/

#define FREQ_SH			16  /* 16.16 fixed point (frequency calculations) */
#define EG_SH			16  /* 16.16 fixed point (envelope generator timing) */
#define LFO_SH			24  /*  8.24 fixed point (LFO calculations)       */
#define TIMER_SH		16  /* 16.16 fixed point (timers calculations)    */

#define FREQ_MASK		((1<<FREQ_SH)-1)

#define ENV_BITS		10
#define ENV_LEN			(1<<ENV_BITS)
#define ENV_STEP		(128.0/ENV_LEN)

#define MAX_ATT_INDEX	(ENV_LEN-1) /* 1023 */
#define MIN_ATT_INDEX	(0)			/* 0 */

#define EG_ATT			4
#define EG_DEC			3
#define EG_SUS			2
#define EG_REL			1
#define EG_OFF			0

#define SIN_BITS		10
#define SIN_LEN			(1<<SIN_BITS)
#define SIN_MASK		(SIN_LEN-1)

#define TL_RES_LEN		(256) /* 8 bits addressing (real chip) */


#define FINAL_SH	(0)
#define MAXOUT		(+32767)
#define MINOUT		(-32768)


/*  TL_TAB_LEN is calculated as:
*   13 - sinus amplitude bits     (Y axis)
*   2  - sinus sign bit           (Y axis)
*   TL_RES_LEN - sinus resolution (X axis)
*/
#define TL_TAB_LEN (13*2*TL_RES_LEN)

#define ENV_QUIET		(TL_TAB_LEN>>3)


/* for busy flag emulation , function FM_GET_TIME_NOW() should be */
/* return the present time in second unit with (double) value     */
/* in timer.c */
#define FM_GET_TIME_NOW() timer_get_time()

typedef INT16 FMSAMPLE;
typedef INT32 FMSAMPLE_MIX;

typedef void (*FM_TIMERHANDLER)(int channel, int count, double stepTime);
typedef void (*FM_IRQHANDLER)(int irq);

void YM2610Init(int baseclock, void *pcmroma, int pcmsizea,
#if (EMU_SYSTEM == MVS)
				void *pcmromb, int pcmsizeb,
#endif
				FM_TIMERHANDLER TimerHandler,
				FM_IRQHANDLER IRQHandler);

void YM2610Reset(void);
int YM2610Write(int addr, UINT8 value);
UINT8 YM2610Read(int addr);
int YM2610TimerOver(int channel);
void YM2610_set_samplerate(void);

#ifdef SAVE_STATE
STATE_SAVE( ym2610 );
STATE_LOAD( ym2610 );
#endif

/* struct describing a single operator (SLOT) */
typedef struct
{
	INT32 *DT;		/* detune          :dt_tab[DT] */
	UINT8  KSR;		/* key scale rate  :3-KSR */
	UINT32 ar;			/* attack rate  */
	UINT32 d1r;		/* decay rate   */
	UINT32 d2r;		/* sustain rate */
	UINT32 rr;			/* release rate */
	UINT8  ksr;		/* key scale rate  :kcode>>(3-KSR) */
	UINT32 mul;		/* multiple        :ML_TABLE[ML] */

	/* Phase Generator */
	UINT32 phase;		/* phase counter */
	UINT32 Incr;		/* phase step */

	/* Envelope Generator */
	UINT8  state;		/* phase type */
	UINT32 tl;			/* total level: TL << 3 */
	INT32 volume;		/* envelope counter */
	UINT32 sl;			/* sustain level:sl_table[SL] */
	UINT32 vol_out;	/* current output from EG circuit (without AM from LFO) */

	UINT8  eg_sh_ar;	/*  (attack state) */
	UINT8  eg_sel_ar;	/*  (attack state) */
	UINT8  eg_sh_d1r;	/*  (decay state) */
	UINT8  eg_sel_d1r;	/*  (decay state) */
	UINT8  eg_sh_d2r;	/*  (sustain state) */
	UINT8  eg_sel_d2r;	/*  (sustain state) */
	UINT8  eg_sh_rr;	/*  (release state) */
	UINT8  eg_sel_rr;	/*  (release state) */

	UINT8  ssg;		/* SSG-EG waveform */
	UINT8  ssgn;		/* SSG-EG negated output */

	UINT32 key;		/* 0=last key was KEY OFF, 1=KEY ON */

	/* LFO */
	UINT32 AMmask;		/* AM enable flag */

} FM_SLOT;

typedef struct
{
	FM_SLOT SLOT[4];	/* four SLOTs (operators) */

	UINT8  ALGO;			/* algorithm */
	UINT8  FB;				/* feedback shift */
	INT32 op1_out[2];		/* op1 output for feedback */

	INT32 *connect1;		/* SLOT1 output pointer */
	INT32 *connect3;		/* SLOT3 output pointer */
	INT32 *connect2;		/* SLOT2 output pointer */
	INT32 *connect4;		/* SLOT4 output pointer */

	INT32 *mem_connect;	/* where to put the delayed sample (MEM) */
	INT32 mem_value;		/* delayed sample (MEM) value */

	INT32 pms;			/* channel PMS */
	UINT8  ams;			/* channel AMS */

	UINT32 fc;				/* fnum,blk:adjusted to sample rate */
	UINT8  kcode;			/* key code:                        */
	UINT32 block_fnum;		/* current blk/fnum value for this slot (can be different betweeen slots of one channel in 3slot mode) */
} FM_CH;


typedef struct
{
	int clock;			/* master clock  (Hz)   */
	int rate;			/* sampling rate (Hz)   */
	float freqbase;		/* frequency base       */
	float TimerBase;	/* Timer base time      */
#if FM_BUSY_FLAG_SUPPORT
	float BusyExpire;	/* ExpireTime of Busy clear */
#endif
	UINT8  address;		/* address register     */
	UINT8  irq;			/* interrupt level      */
	UINT8  irqmask;		/* irq mask             */
	UINT8  status;			/* status flag          */
	UINT32 mode;			/* mode  CSM / 3SLOT    */
	UINT8  prescaler_sel;	/* prescaler selector */
	UINT8  fn_h;			/* freq latch           */
	INT32 TA;				/* timer a              */
	INT32 TAC;			/* timer a counter      */
	UINT8  TB;				/* timer b              */
	INT32 TBC;			/* timer b counter      */
	/* local time tables */
	INT32 dt_tab[8][32];	/* DeTune table       */
	/* Extention Timer and IRQ handler */
	FM_TIMERHANDLER Timer_Handler;
	FM_IRQHANDLER   IRQ_Handler;
} FM_ST;


/* OPN 3slot struct */
typedef struct
{
	UINT32 fc[3];			/* fnum3,blk3: calculated */
	UINT8  fn_h;			/* freq3 latch */
	UINT8  kcode[3];		/* key code */
	UINT32 block_fnum[3];	/* current fnum value for this slot (can be different betweeen slots of one channel in 3slot mode) */
} FM_3SLOT;

/* OPN/A/B common state */
typedef struct
{
	FM_ST ST;				/* general state */
	FM_3SLOT SL3;			/* 3 slot mode state */
	FM_CH *P_CH;			/* pointer of CH */
	UINT32 pan[6*2];			/* fm channels output masks (0xffffffff = enable) */

	UINT32 eg_cnt;				/* global envelope generator counter */
	UINT32 eg_timer;			/* global envelope generator counter works at frequency = chipclock/64/3 */
	UINT32 eg_timer_add;		/* step of eg_timer */
	UINT32 eg_timer_overflow;	/* envelope generator timer overlfows every 3 samples (on real chip) */

	/* there are 2048 FNUMs that can be generated using FNUM/BLK registers
        but LFO works with one more bit of a precision so we really need 4096 elements */

	UINT32 fn_table[4096];		/* fnumber->increment counter */

	/* LFO */
	UINT32 lfo_cnt;
	UINT32 lfo_inc;

	UINT32 lfo_freq[8];		/* LFO FREQ table */
} FM_OPN;




/* ADPCM type A channel struct */
typedef struct
{
	UINT8		flag;			/* port state				*/
	UINT8		flagMask;		/* arrived flag mask		*/
	UINT8		now_data;		/* current ROM data			*/
	UINT32		now_addr;		/* current ROM address		*/
	UINT32		now_step;
	UINT32		step;
	UINT32		start;			/* sample data start address*/
	UINT32		end;			/* sample data end address	*/
	UINT8		IL;				/* Instrument Level			*/
	INT32		adpcma_acc;		/* accumulator				*/
	INT32		adpcma_step;	/* step						*/
	INT32		adpcma_out;		/* (speedup) hiro-shi!!		*/
	INT8		vol_mul;		/* volume in "0.75dB" steps	*/
	UINT8		vol_shift;		/* volume in "-6dB" steps	*/
	INT32		*pan;			/* &out_adpcma[OPN_xxxx] 	*/

#if (EMU_SYSTEM == MVS) && !defined(PSP_SLIM)
	UINT16		block;
	UINT8		*buf;
#endif

} ADPCMA;


#if (EMU_SYSTEM == MVS)
/* ADPCM type B struct */
typedef struct adpcmb_state
{
	INT32		*pan;			/* pan : &output_pointer[pan]   */
	float	freqbase;
	int		output_range;
	UINT32		now_addr;		/* current address      */
	UINT32		now_step;		/* currect step         */
	UINT32		step;			/* step                 */
	UINT32		start;			/* start address        */
	UINT32		limit;			/* limit address        */
	UINT32		end;			/* end address          */
	UINT32		delta;			/* delta scale          */
	INT32		volume;			/* current volume       */
	INT32		acc;			/* shift Measurement value*/
	INT32		adpcmd;			/* next Forecast        */
	INT32		adpcml;			/* current value        */
	INT32		prev_acc;		/* leveling value       */
	UINT8		now_data;		/* current rom data     */
	UINT8		CPU_data;		/* current data from reg 08 */
	UINT8		portstate;		/* port status          */

#ifndef PSP_SLIM
	UINT16		block;
	UINT8		*buf;
#endif

	/* note that different chips have these flags on different
    ** bits of the status register
    */
	UINT8		status_change_EOS_bit;		/* 1 on End Of Sample (record/playback/cycle time of AD/DA converting has passed)*/
	UINT8		status_change_BRDY_bit;		/* 1 after recording 2 datas (2x4bits) or after reading/writing 1 data */

	/* neither Y8950 nor YM2608 can generate IRQ when PCMBSY bit changes, so instead of above,
    ** the statusflag gets ORed with PCM_BSY (below) (on each read of statusflag of Y8950 and YM2608)
    */
	UINT8		PCM_BSY;		/* 1 when ADPCM is playing; Y8950/YM2608 only */
} ADPCMB;
#endif




/* log output level */
#define LOG_ERR  3      /* ERROR       */
#define LOG_WAR  2      /* WARNING     */
#define LOG_INF  1      /* INFORMATION */
#define LOG_LEVEL LOG_INF

#define LOG(n,x) if( (n)>=LOG_LEVEL ) logerror x

#ifdef NO_INLINE
/*********************************************************************************************/
// formerly inline functions
void FM_STATUS_SET(FM_ST *ST,int flag);
void FM_STATUS_RESET(FM_ST *ST,int flag);
void FM_IRQMASK_SET(FM_ST *ST,int flag);
void set_timers( FM_ST *ST, int v );
void TimerAOver(FM_ST *ST);
void TimerBOver(FM_ST *ST);
UINT8 FM_STATUS_FLAG(FM_ST *ST);
void FM_BUSY_SET(FM_ST *ST,int busyclock );
void FM_KEYON(FM_CH *CH , int s );
void FM_KEYOFF(FM_CH *CH , int s );
void set_det_mul(FM_ST *ST,FM_CH *CH,FM_SLOT *SLOT,int v);
void set_tl(FM_CH *CH,FM_SLOT *SLOT , int v);
void set_tl(FM_CH *CH,FM_SLOT *SLOT , int v);
void set_ar_ksr(FM_CH *CH,FM_SLOT *SLOT,int v);
void set_dr(FM_SLOT *SLOT,int v);
void set_sr(FM_SLOT *SLOT,int v);
void set_sl_rr(FM_SLOT *SLOT,int v);
void advance_lfo(FM_OPN *OPN);
void advance_eg_channel(FM_OPN *OPN, FM_SLOT *SLOT);
INT32 op_calc(UINT32 phase, UINT32 env, INT32 pm);
void chan_calc(FM_OPN *OPN, FM_CH *CH);
void refresh_fc_eg_slot(FM_SLOT *SLOT , int fc , int kc );
void refresh_fc_eg_chan(FM_CH *CH);
void CSMKeyControll(FM_CH *CH);
#endif

#endif /* _YM2610_H_ */
