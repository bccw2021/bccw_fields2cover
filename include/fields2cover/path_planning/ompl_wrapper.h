//=============================================================================
//    Copyright (C) 2021-2024 Wageningen University - All Rights Reserved
//                     Author: Gonzalo Mier
//                        BSD-3 License
//=============================================================================

#pragma once
#ifndef FIELDS2COVER_PATH_PLANNING_OMPL_WRAPPER_H_
#define FIELDS2COVER_PATH_PLANNING_OMPL_WRAPPER_H_

// 先定义兼容C++11的OMPL宏
// 这些宏将覆盖OMPL库中的宏定义
#define OMPL_ERROR(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_ERROR, fmt, ## __VA_ARGS__)
#define OMPL_WARN(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_WARN, fmt, ## __VA_ARGS__)
#define OMPL_INFORM(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_INFO, fmt, ## __VA_ARGS__)
#define OMPL_DEBUG(fmt, ...) ompl::msg::log(__FILE__, __LINE__, ompl::msg::LOG_DEBUG, fmt, ## __VA_ARGS__)

// 包含OMPL头文件
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

#endif  // FIELDS2COVER_PATH_PLANNING_OMPL_WRAPPER_H_
