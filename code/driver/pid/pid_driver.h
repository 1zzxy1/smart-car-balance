#ifndef __PID_H
#define __PID_H

/* pid缁撴瀯浣?*/
typedef struct
{
    float kp;                    /* 姣斾緥 */
    float ki;                    /* 绉垎 */
    float kd;                    /* 寰垎 */
    float target;                /* 鐩爣鍊?*/
    float current;               /* 褰撳墠鍊?*/
    float out;                   /* 鎵ц閲?*/
    float limit;                /* PID(out)杈撳嚭闄愬箙鍊?*/

    float error;                 /* 褰撳墠璇樊 */
    float last_error;            /* 涓婁竴娆¤宸?*/
    float last2_error;           /* 涓婁笂娆¤宸?*/
    float last_out;              /* 涓婁竴娆℃墽琛岄噺 */
    float integral;              /* 绉垎锛堢疮鍔狅級 */
    float p_out, i_out, d_out;   /* 姣斾緥銆佺Н鍒嗐€佸井鍒嗗€?*/
} PID_T;

/*
    鎻愪緵缁欑敤鎴疯皟鐢ㄧ殑API
*/
/* PID鍒濆鍖?*/
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit);

/* 璁剧疆PID鐩爣鍊?*/
void pid_set_target(PID_T * _tpPID, float _target);

/* 璁剧疆PID鍙傛暟 */
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd);

/* 璁剧疆PID杈撳嚭闄愬箙 */
void pid_set_limit(PID_T * _tpPID, float _limit);

/* 閲嶇疆PID鎺у埗鍣?*/
void pid_reset(PID_T * _tpPID);

/* 璁＄畻浣嶇疆寮廝ID */
float pid_calculate_positional(PID_T * _tpPID, float _current);

/* 璁＄畻澧為噺寮廝ID */
float pid_calculate_incremental(PID_T * _tpPID, float _current);

/* 鏍规嵁澶栭儴璇樊璁＄畻PID杈撳嚭 */
float pid_calculate_by_error(PID_T * _tpPID, float _error);

/* 闄愬箙鍑芥暟 */
float pid_constrain(float value, float min, float max);

/* 绉垎闄愬箙鍑芥暟 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max);

#endif
