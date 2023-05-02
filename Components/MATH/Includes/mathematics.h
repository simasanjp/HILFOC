/* Copyright (c) 2020 eonas IT-Beratung und -Entwicklung GmbH All rights reserved. */
/**
 * \file math.h
 *
 * \brief  Generic mathematical functions.
 */
#ifndef __MATH_H_
#define __MATH_H_
#include "netx_drv.h"
/**
 * \brief Size of the Sin table.
 */
#define MATH_SINUS_TABLE_SIZE (uint16_t)256
/**
 * \brief pi over 2 in digits.
 */
#define MATH_PIOVER2_DIG      (uint16_t)(65535/4)
/**
 * \brief Conversation factor from Angle to table index.
 */
#define MATH_SINTABLE_INDEXER (uint16_t)(256)
/**
 * \brief Square-root of 3.
 */
#define MATH_SQRT_3           1.73205080757f
/**
 * \brief Square-root of 3 over 2.
 */
#define MATH_SQRT3_OVER_2     (MATH_SQRT_3/2.0f)
/**
 * \brief One over square-root of 3.
 */
#define MATH_ONE_OVER_SQRT3   (1.0f/MATH_SQRT_3)

/**
 * \brief 2D-vector for Sin and Cos
 */
typedef struct MATH_Vector_SC_S
{
  float f_sin; /**< Sin */
  float f_cos; /**< Cos */
} Math_Vector_SC_T;
/**
 * \brief 2D-vector for alpha/beta domain
 */
typedef struct MATH_Vector_AB_S
{
  float f_alpha; /**< alpha */
  float f_beta; /**< beta */
} Math_Vector_AB_T;

/**
 * \brief 2D-vector for d/q domain
 */
typedef struct MATH_Vector_DQ_S
{
  float f_d; /**< d */
  float f_q; /**< q */
} Math_Vector_DQ_T;

/**
 * \brief 3D-vector in UVW domain
 */
typedef struct
{
  float f_u; /**< phase U */
  float f_v; /**< phase V */
  float f_w; /**< phase W */
} Math_Vector_UVW_T;

/**
 * \brief PI Controller state
 */
typedef struct
{
  float f_integral; /**< Integral part*/
} Math_PI_Controller_State_T;

/**
 * \brief PI controller parameters
 */
typedef struct
{
  float f_LowerLimit; /**< lower limit */
  float f_UpperLimit; /**< upper limit */
  float f_KP; /**< proportional gain */
  float f_KI; /**< integral gain */
} Math_PI_Controller_Parameter_T;

/**
 * \brief pi controller state
 *
 * The pi controller just contains an integral state in Q1.31 notation.
 */
typedef struct MATH_Fixedpt_PI_Controller_State_S {
  int32_t s32_i_n;
} MATH_Fixedpt_PI_Controller_State_T;

/**
 * \brief pi controller parameters
 *
 * \warning s32_LowerLimit must be smaller than s32_UpperLimit
 */
typedef struct MATH_Fixedpt_PI_Controller_ParameterS {
  int32_t s32_LowerLimit; /**< lower limit for y and integral part in Q1.31 */
  int32_t s32_UpperLimit; /**< upper limit for y and integral part in Q1.31 */
  int32_t s32_KP;         /**< proportional gain in Q16.16 */
  int32_t s32_KI;         /**< integral gain in Q16.16 */
} MATH_Fixedpt_PI_Controller_Parameter_T;
/**
 * \brief Float sin table.
 */
extern const float f_SinusTable256[MATH_SINUS_TABLE_SIZE ];


/**
 * \brief         Returns sin of 16-bit angle.
 * @param[in]     au16_angle angle in uint16 scale
 */
float Math_Sinf(uint16_t au16_angle);

/**
 * \brief         Returns cos of 16-bit angle.
 * @param[in]     au16_angle angle in uint16 scale
 */
float Math_Cosf(uint16_t au16_angle);

static inline int16_t Math_s32_Convert_s16(int32_t as32_In) {
  return ((int16_t) (as32_In >> 16));
}
static inline int16_t Math_s16_Convert_s32(int16_t as16_In) {
  int32_t ls32_In;

  ls32_In = as16_In;
  return ls32_In << 16;
}
/**
 * \brief absolute value of an int16_t
 * \param as16_In input value
 * \return absolute of the input value, |input value|
 */
static inline int16_t Math_s16_Abs(int16_t as16_In) {
  int16_t ls16_Ret;

  if (as16_In >= 0) {
    ls16_Ret = as16_In;
  } else {
    ls16_Ret = -as16_In;
  }
  return ls16_Ret;
}
/**
 * \brief         Returns sin and cos of 16-bit angle.
 * @param[in]     au16_angle angle in uint16 scale
 */
static inline Math_Vector_SC_T Math_SinCos(uint16_t au16_angle)
{
  Math_Vector_SC_T ls_Return;
  uint16_t indexSin;
  uint16_t indexCos;
  uint16_t cosangle = (uint16_t) (au16_angle + MATH_PIOVER2_DIG );

  indexCos = (uint16_t) (cosangle / MATH_SINTABLE_INDEXER );
  indexSin = (uint16_t) (au16_angle / MATH_SINTABLE_INDEXER );

  ls_Return.f_sin = f_SinusTable256[indexSin];
  ls_Return.f_cos = f_SinusTable256[indexCos];

  return ls_Return;
}

/**
 * \brief         Inverse Park transformation.
 * @param[in]     *aps_DQVector pointer to vector in DQ frame.
 * @param[in]     *aps_SinCos   pointer to sin and cos vector.
 */
static inline Math_Vector_AB_T Math_InversePark(Math_Vector_DQ_T *aps_DQVector, Math_Vector_SC_T *aps_SinCos)
{
  Math_Vector_AB_T l_Result;

  l_Result.f_alpha = (aps_DQVector->f_d * aps_SinCos->f_cos) - (aps_DQVector->f_q * aps_SinCos->f_sin);
  l_Result.f_beta = (aps_DQVector->f_q * aps_SinCos->f_cos) + (aps_DQVector->f_d * aps_SinCos->f_sin);

  return l_Result;
}

/**
 * \brief         Inverse Clarke transformation.
 * @param[in]     *aps_ABVector   pointer to vector in alpha-beta frame.
 */
static inline Math_Vector_UVW_T Math_InverseClarke(Math_Vector_AB_T *aps_ABVector)
{
  Math_Vector_UVW_T l_Result;

  l_Result.f_u = aps_ABVector->f_alpha;
  l_Result.f_v = ((-aps_ABVector->f_alpha + (MATH_SQRT_3 * aps_ABVector->f_beta)) / 2.0f);
  l_Result.f_w = -l_Result.f_u - l_Result.f_v;

  return l_Result;
}

/**
 * \brief Clarke transformation.
 * @param[in]     *aps_UVWVector   pointer to vector in UVW frame.
 */
static inline Math_Vector_AB_T Math_Clarke(Math_Vector_UVW_T *aps_UVWVector)
{
  Math_Vector_AB_T l_Result;

  l_Result.f_alpha = (2.0f * aps_UVWVector->f_u - aps_UVWVector->f_v - aps_UVWVector->f_w) / 3.0f;
  l_Result.f_beta = MATH_ONE_OVER_SQRT3 * (aps_UVWVector->f_v - aps_UVWVector->f_w);

  return l_Result;
}

/**
 * \brief Park transformation.
 * @param[in]     *aps_ABVector   pointer to vector in alpha-beta frame.
 * @param[in]     *aps_SinCos   pointer to sin and cos vector.
 */
static inline Math_Vector_DQ_T Math_Park(Math_Vector_AB_T *aps_ABVector, Math_Vector_SC_T *aps_SinCos)
{
  Math_Vector_DQ_T l_Result;

  l_Result.f_d = (aps_SinCos->f_cos * aps_ABVector->f_alpha) + (aps_SinCos->f_sin * aps_ABVector->f_beta);
  l_Result.f_q = (aps_SinCos->f_cos * aps_ABVector->f_beta) - (aps_SinCos->f_sin * aps_ABVector->f_alpha);

  return l_Result;
}

/**
 * \brief pi controller function with float input/output
 *
 * The function takes an input error value and returns a new control value. The parallel form of the pi controller is used.
 * The limits are applied internally to the integral part and the control value.
 *
 * @param[in,out] aps_State pi controller state
 * @param[in]     as_Parameter pi controller parameter
 * @param[in]     af_Error difference between set point and measurement
 * @return        updated control value
 */
static inline float Math_PIController(Math_PI_Controller_State_T *aps_State, const Math_PI_Controller_Parameter_T *as_Parameter, float af_Error)
{
  float f_ErrorKp;
  float f_ErrorKi;
  float f_i_n;
  float f_y_n;

  f_ErrorKp = as_Parameter->f_KP * af_Error;
  f_ErrorKi = as_Parameter->f_KI * af_Error;
  f_i_n = aps_State->f_integral + f_ErrorKi;

  if(f_i_n > as_Parameter->f_UpperLimit)
  {
    f_i_n = as_Parameter->f_UpperLimit;
  }
  if(f_i_n < as_Parameter->f_LowerLimit)
  {
    f_i_n = as_Parameter->f_LowerLimit;
  }

  f_y_n = f_i_n + f_ErrorKp;

  if(f_y_n > as_Parameter->f_UpperLimit)
  {
    f_y_n = as_Parameter->f_UpperLimit;
  }
  if(f_y_n < as_Parameter->f_LowerLimit)
  {
    f_y_n = as_Parameter->f_LowerLimit;
  }
  aps_State->f_integral = f_i_n;

  return f_y_n;
}
/**
 * \brief pi controller worker function with Q1.15 input range
 *
 * The function takes an input error value and returns a new control value. The parallel form of the pi controller is used.
 * The limits are applied internally to the integral part and the control value.
 *
 * \warning s32_LowerLimit must be <= s32_UpperLimit
 * \warning The product of the input range and the gain is limited to 31 bit in the integer domain. It silently overflows when bigger values are processed.
 *
 * \param[in,out] aps_State pi controller state
 * \param[in]     as_Parameter pi controller parameter
 * \param[in]     as16_Error difference between set point and measurement in Q1.15
 * \return        updated control value in Q1.15
 */
static inline int16_t MATH_Fixedpt_PI_Controller(MATH_Fixedpt_PI_Controller_State_T *aps_State,
                                                const MATH_Fixedpt_PI_Controller_Parameter_T *as_Parameter,
                                                int16_t as16_Error) {
  int32_t ls32_ErrorKp;
  int32_t ls32_ErrorKi;
  int32_t ls32_y_n;
  int32_t ls32_UpperLimit;
  int32_t ls32_LowerLimit;
  int32_t ls32_i_n;

  ls32_ErrorKp = as_Parameter->s32_KP * ((int32_t) as16_Error);
  ls32_ErrorKi = as_Parameter->s32_KI * ((int32_t) as16_Error);

  ls32_i_n = aps_State->s32_i_n + ls32_ErrorKi;
  ls32_y_n = ls32_i_n + ls32_ErrorKp;

  ls32_UpperLimit = as_Parameter->s32_UpperLimit;
  if (ls32_i_n > ls32_UpperLimit) {
    ls32_i_n = ls32_UpperLimit;
  }
  if (ls32_y_n > ls32_UpperLimit) {
    ls32_y_n = ls32_UpperLimit;
  }

  ls32_LowerLimit = as_Parameter->s32_LowerLimit;
  if (ls32_i_n < ls32_LowerLimit) {
    ls32_i_n = ls32_LowerLimit;
  }

  if (ls32_y_n < ls32_LowerLimit) {
    ls32_y_n = ls32_LowerLimit;
  }

  aps_State->s32_i_n = ls32_i_n;
  return Math_s32_Convert_s16(ls32_y_n);
}

/**
 * \brief         Space vector modulation.
 * \details       Converts sinusoidal uvw to space vector uvw.
 * @param[in]     *aps_Vector pointer to vector in UVW frame.
 */
static inline Math_Vector_UVW_T Math_SVM(Math_Vector_UVW_T *aps_Vector)
{
  Math_Vector_UVW_T l_Result;

  float lf_MaxDuty = aps_Vector->f_u;
  float lf_MinDuty = aps_Vector->f_u;
  float lf_Teffective;
  float lf_Toffset;

  /* Max Min sorting algorithm */
  if(aps_Vector->f_v > lf_MaxDuty)
  {
    lf_MaxDuty = aps_Vector->f_v;
  }
  if(aps_Vector->f_v < lf_MinDuty)
  {
    lf_MinDuty = aps_Vector->f_v;
  }

  if(aps_Vector->f_w > lf_MaxDuty)
  {
    lf_MaxDuty = aps_Vector->f_w;
  }
  if(aps_Vector->f_w < lf_MinDuty)
  {
    lf_MinDuty = aps_Vector->f_w;
  }

  /* Effective time calculation */
  lf_Teffective = lf_MaxDuty + lf_MinDuty;
  lf_Toffset = lf_Teffective / 2;

  l_Result.f_u = (aps_Vector->f_u - lf_Toffset);
  l_Result.f_v = (aps_Vector->f_v - lf_Toffset);
  l_Result.f_w = (aps_Vector->f_w - lf_Toffset);

  return l_Result;
}

/**
 * \brief         Square-root function.
 * \details       Calculates square-root based on Babylonian method.
 * @param[in]     af_Square   Input value for square-root function.
 * @param[in]     af_Guess    Initial square-root guess.
 * @param[in]     au8_it      Number of iterations.
 */
static inline float Math_Sqrt(float af_Square, float af_Guess, uint8_t au8_it)
{
  float fl_Return = 0.0f;
  float fl_PrevGuess;
  fl_PrevGuess = af_Guess;

  for(uint_fast8_t i = 0; i < au8_it; ++i)
  {
    fl_Return = (fl_PrevGuess + (af_Square / fl_PrevGuess)) / 2;
    fl_PrevGuess = fl_Return;
  }

  return fl_Return;
}

/**
 * \brief         Median filter with 3 elements.
 * @param[in]     *ap_Values      Pointer to array of values.
 */
static inline int32_t Math_MedianFilter(int32_t *ap_Values)
{
  int32_t ls32_Middle = 0;

  if((ap_Values[0] <= ap_Values[1]) && (ap_Values[0] <= ap_Values[2]))
  {
    ls32_Middle = (ap_Values[1] <= ap_Values[2]) ? ap_Values[1] : ap_Values[2];
  }
  else if((ap_Values[1] <= ap_Values[0]) && (ap_Values[1] <= ap_Values[2]))
  {
    ls32_Middle = (ap_Values[0] <= ap_Values[2]) ? ap_Values[0] : ap_Values[2];
  }
  else
  {
    ls32_Middle = (ap_Values[0] <= ap_Values[1]) ? ap_Values[0] : ap_Values[1];
  }

  return ls32_Middle;
}

static inline void MATH_LowPassFilter_Init(int32_t *aps32_FilterState, int16_t as16_RawValue) {
  *aps32_FilterState = ((int32_t) as16_RawValue) << 16;
}

static inline int16_t MATH_LowPassFilter(int32_t *aps32_FilterState, int16_t as16_RawValue,
                                             uint16_t au16_FilterFactor) {
  (*aps32_FilterState) += (int32_t) (((int32_t) (as16_RawValue - Math_s32_Convert_s16(*aps32_FilterState))) *
      (int32_t) (au16_FilterFactor));
  return Math_s32_Convert_s16(*aps32_FilterState);
}
/**
 * \brief         Check if given values within the given bounds
 * @param[in]     au16_Value1, au16_Value2, au16_Limit
 */
static inline int8_t Math_QuasyEqual(uint16_t au16_Value1, uint16_t au16_Value2, uint16_t au16_Limit)
{
  int8_t ls8_Return = 0;

  if (au16_Value1 > au16_Value2){
    if((au16_Value1 - au16_Value2) < au16_Limit){
      ls8_Return = 1;
    }
  }
  else if (au16_Value2 > au16_Value1){
    if((au16_Value2 - au16_Value1) < au16_Limit){
      ls8_Return = 1;
    }
  }
  else
  {
    ls8_Return = 1;
  }

  return ls8_Return;
}
#endif /* #ifndef __MATH_H_ */
