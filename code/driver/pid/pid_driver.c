#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid_driver.h"

/* 鍐呴儴鍔熻兘鍑芥暟 */
static void pid_formula_incremental(PID_T * _tpPID);
static void pid_formula_positional(PID_T * _tpPID);
static void pid_out_limit(PID_T * _tpPID);

/*******************************************************************************
 * @brief PID鍒濆鍖栧嚱鏁帮紝鐢ㄤ簬鍒濆鍖朠ID缁撴瀯浣?
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _kp 姣斾緥绯绘暟
 * @param {float} _ki 绉垎绯绘暟
 * @param {float} _kd 寰垎绯绘暟
 * @param {float} _target 鐩爣鍊?
 * @param {float} _limit 杈撳嚭闄愬箙鍊?
 * @return {*}
 * @note 浣跨敤鍓嶅繀椤昏皟鐢ㄨ鍑芥暟鍒濆鍖朠ID鍙傛暟
 *******************************************************************************/
void pid_init(PID_T * _tpPID, float _kp, float _ki, float _kd, float _target, float _limit)
{
    _tpPID->kp = _kp;          // 姣斾緥
    _tpPID->ki = _ki;          // 绉垎
    _tpPID->kd = _kd;          // 寰垎
    _tpPID->target = _target;  // 鐩爣鍊?
    _tpPID->limit = _limit;    // 闄愬箙鍊?
    _tpPID->integral = 0;      // 绉垎椤规竻闆?
    _tpPID->last_error = 0;    // 涓婃璇樊娓呴浂
    _tpPID->last2_error = 0;   // 涓婁笂娆¤宸竻闆?
    _tpPID->out = 0;           // 杈撳嚭鍊兼竻闆?
    _tpPID->p_out = 0;         // P杈撳嚭娓呴浂
    _tpPID->i_out = 0;         // I杈撳嚭娓呴浂
    _tpPID->d_out = 0;         // D杈撳嚭娓呴浂
}

/*******************************************************************************
 * @brief 璁剧疆PID鐩爣鍊?
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _target 鐩爣鍊?
 * @return {*}
 * @note 鐢ㄤ簬鍔ㄦ€佽皟鏁碢ID鎺у埗鍣ㄧ殑鐩爣鍊?
 *******************************************************************************/
void pid_set_target(PID_T * _tpPID, float _target)
{
    _tpPID->target = _target;
}

/*******************************************************************************
 * @brief 璁剧疆PID鍙傛暟
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _kp 姣斾緥绯绘暟
 * @param {float} _ki 绉垎绯绘暟
 * @param {float} _kd 寰垎绯绘暟
 * @return {*}
 * @note 鐢ㄤ簬鍔ㄦ€佽皟鏁碢ID鍙傛暟
 *******************************************************************************/
void pid_set_params(PID_T * _tpPID, float _kp, float _ki, float _kd)
{
    _tpPID->kp = _kp;
    _tpPID->ki = _ki;
    _tpPID->kd = _kd;
}

/*******************************************************************************
 * @brief 璁剧疆PID杈撳嚭闄愬箙
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _limit 闄愬箙鍊?
 * @return {*}
 * @note 鐢ㄤ簬鍔ㄦ€佽皟鏁碢ID杈撳嚭闄愬箙
 *******************************************************************************/
void pid_set_limit(PID_T * _tpPID, float _limit)
{
    _tpPID->limit = _limit;
}

/*******************************************************************************
 * @brief 閲嶇疆PID鎺у埗鍣?
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @return {*}
 * @note 娓呴櫎鎵€鏈夊巻鍙茶宸暟鎹?
 *******************************************************************************/
void pid_reset(PID_T * _tpPID)
{
    _tpPID->integral = 0;
    _tpPID->last_error = 0;
    _tpPID->last2_error = 0;
    _tpPID->out = 0;
    _tpPID->p_out = 0;
    _tpPID->i_out = 0;
    _tpPID->d_out = 0;
}

/*******************************************************************************
 * @brief 璁＄畻浣嶇疆寮廝ID
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _current 褰撳墠鍊?
 * @return {float} PID璁＄畻鍚庣殑杈撳嚭鍊?
 * @note 浣嶇疆寮廝ID锛歅-鍝嶅簲鎬э紝I-鍑嗙‘鎬э紝D-绋冲畾鎬?
 *******************************************************************************/
float pid_calculate_positional(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_positional(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/*******************************************************************************
 * @brief 璁＄畻澧為噺寮廝ID
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @param {float} _current 褰撳墠鍊?
 * @return {float} PID璁＄畻鍚庣殑杈撳嚭鍊?
 * @note 澧為噺寮廝ID锛歅-绋冲畾鎬э紝I-鍝嶅簲鎬э紝D-鍑嗙‘鎬?
 *******************************************************************************/
float pid_calculate_incremental(PID_T * _tpPID, float _current)
{
    _tpPID->current = _current;
    pid_formula_incremental(_tpPID);
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

float pid_calculate_by_error(PID_T * _tpPID, float _error)
{
    _tpPID->error = _error;
    _tpPID->integral += _tpPID->error;

    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = _tpPID->ki * _tpPID->integral;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);

    _tpPID->out = _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;

    _tpPID->last_error = _tpPID->error;
    pid_out_limit(_tpPID);
    return _tpPID->out;
}

/* 鈥斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€?PID鐩稿叧鐨勫姛鑳藉嚱鏁?鈥斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€斺€?*/
/*******************************************************************************
 * @brief 杈撳嚭闄愬箙鍑芥暟
 * @param {PID_T *} _tpPID 鎸囧悜PID缁撴瀯浣撶殑鎸囬拡
 * @return {*}
 * @note 闃叉杈撳嚭瓒呭嚭闄愬畾鑼冨洿
 *******************************************************************************/
static void pid_out_limit(PID_T * _tpPID)
{
    if(_tpPID->out > _tpPID->limit)
        _tpPID->out = _tpPID->limit;
    else if(_tpPID->out < -_tpPID->limit)
        _tpPID->out = -_tpPID->limit;
}

/*******************************************************************************
 * @brief 澧為噺寮廝ID鍏紡
 * @param {PID_T *} _tpPID  浼犲叆瑕佽绠楃殑PID鍙傛暟鎸囬拡
 * @return {*}
 * @note 鍦ㄥ閲忓紡涓紝P-绋冲畾鎬э紝I-鍝嶅簲鎬э紝D-鍑嗙‘鎬?
 *******************************************************************************/
static void pid_formula_incremental(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;

    _tpPID->p_out = _tpPID->kp * (_tpPID->error - _tpPID->last_error);
    _tpPID->i_out = _tpPID->ki * _tpPID->error;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - 2 * _tpPID->last_error + _tpPID->last2_error);

    _tpPID->out += _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;

    _tpPID->last2_error = _tpPID->last_error;
    _tpPID->last_error = _tpPID->error;
}

/*******************************************************************************
 * @brief 浣嶇疆寮廝ID鍏紡
 * @param {PID_T *} _tpPID  浼犲叆瑕佽绠楃殑PID鍙傛暟鎸囬拡
 * @return {*}
 * @note 鍦ㄤ綅缃紡涓紝P-鍝嶅簲鎬э紝I-鍑嗙‘鎬э紝D-绋冲畾鎬?
 *******************************************************************************/
static void pid_formula_positional(PID_T * _tpPID)
{
    _tpPID->error = _tpPID->target - _tpPID->current;
    _tpPID->integral += _tpPID->error;

    _tpPID->p_out = _tpPID->kp * _tpPID->error;
    _tpPID->i_out = _tpPID->ki * _tpPID->integral;
    _tpPID->d_out = _tpPID->kd * (_tpPID->error - _tpPID->last_error);

    _tpPID->out = _tpPID->p_out + _tpPID->i_out + _tpPID->d_out;

    _tpPID->last_error = _tpPID->error;
}

/**
 * @brief 闄愬箙鍑芥暟
 * @param value 杈撳叆鍊?
 * @param min 鏈€灏忓€?
 * @param max 鏈€澶у€?
 * @return 闄愬箙鍚庣殑鍊?
 */
float pid_constrain(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

/**
 * @brief 绉垎闄愬箙鍑芥暟
 * @param pid PID鎺у埗鍣?
 * @param min 鏈€灏忓€?
 * @param max 鏈€澶у€?
 * @note 鍦ㄤ娇鐢ㄥ閲忓紡PID鎺у埗鏃舵鍑芥暟涓嶉渶瑕侊紝浣嗕负浜嗗彲鑳藉垏鎹㈠洖浣嶇疆寮廝ID鑰屼繚鐣?
 */
void __attribute__((unused)) pid_app_limit_integral(PID_T *pid, float min, float max)
{
    if (pid->integral > max)
    {
        pid->integral = max;
    }
    else if (pid->integral < min)
    {
        pid->integral = min;
    }
}
