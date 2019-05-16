#pragma once
#include <real_t.h>

/** Whether an interrupt is currently running. */
extern volatile bool isInterruptRunning;

/**
 * @file    MainInterrupt.hpp
 * @brief   This file contains an update function that will be called whenever 
 *          the IMU interrupts Main.cpp.
 */

// TODO: comments
real_t calculateYawJump(float yaw);

void mainOperation();

/**
 * @brief   Rather sketchy conversion from a struct of primitives of the same
 *          type to a reference of an array of this type.
 * 
 * @tparam  ArrayElementType
 *          The type of the elements of the result array.
 * @tparam  StructType
 *          The type of the struct to convert. Make sure that the struct is 
 *          well packed, and that the memory layout is consistent with that of
 *          an array. 
 */
template <class ArrayElementType = float, class StructType = void>
static ArrayElementType (&toCppArray(
    StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)];

/**
 * @brief   Rather sketchy conversion from a struct of primitives of the same
 *          type to a reference of an array of this type.
 * 
 * @tparam  ArrayElementType
 *          The type of the elements of the result array.
 * @tparam  StructType
 *          The type of the struct to convert. Make sure that the struct is 
 *          well packed, and that the memory layout is consistent with that of
 *          an array. 
 */
template <class ArrayElementType = float, class StructType = void>
static const ArrayElementType (&toCppArray(
    const StructType &data))[sizeof(StructType) / sizeof(ArrayElementType)];

void updateFSM();