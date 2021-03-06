﻿//------------------------------------------------------------------------------
/// @file
/// @author   ハル研究所プログラミングコンテスト実行委員会
///
/// @copyright  Copyright (c) 2016 HAL Laboratory, Inc.
/// @attention  このファイルの利用は、同梱のREADMEにある
///             利用条件に従ってください。
//------------------------------------------------------------------------------

#pragma once

namespace hpc {

//------------------------------------------------------------------------------
/// 宇宙船の行動の種類。
enum ActionType
{
    ActionType_Move,        // 移動する
    ActionType_Shoot,       // 発射する
    ActionType_TERM,
};

} // namespace
// EOF
