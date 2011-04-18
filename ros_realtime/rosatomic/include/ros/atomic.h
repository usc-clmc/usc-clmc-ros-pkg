/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef ROSATOMIC_ATOMIC_H
#define ROSATOMIC_ATOMIC_H

#include "../../boost_atomic/atomic.hpp"

namespace ros
{

const static boost_atomic::memory_order2 memory_order_relaxed = boost_atomic::memory_order2_relaxed;
const static boost_atomic::memory_order2 memory_order_acquire = boost_atomic::memory_order2_acquire;
const static boost_atomic::memory_order2 memory_order_release = boost_atomic::memory_order2_release;
const static boost_atomic::memory_order2 memory_order_acq_rel = boost_atomic::memory_order2_acq_rel;
const static boost_atomic::memory_order2 memory_order_seq_cst = boost_atomic::memory_order2_seq_cst;
const static boost_atomic::memory_order2 memory_order_consume = boost_atomic::memory_order2_consume;
typedef boost_atomic::memory_order2 memory_order;
using boost_atomic::atomic;
using boost_atomic::atomic_flag;
using boost_atomic::atomic_thread_fence;
using boost_atomic::atomic_char;
using boost_atomic::atomic_uchar;
using boost_atomic::atomic_schar;
using boost_atomic::atomic_uint8_t;
using boost_atomic::atomic_int8_t;
using boost_atomic::atomic_ushort;
using boost_atomic::atomic_short;
using boost_atomic::atomic_uint16_t;
using boost_atomic::atomic_int16_t;
using boost_atomic::atomic_uint;
using boost_atomic::atomic_int;
using boost_atomic::atomic_uint32_t;
using boost_atomic::atomic_int32_t;
using boost_atomic::atomic_ulong;
using boost_atomic::atomic_long;
using boost_atomic::atomic_uint64_t;
using boost_atomic::atomic_int64_t;
using boost_atomic::atomic_ullong;
using boost_atomic::atomic_llong;
using boost_atomic::atomic_address;
using boost_atomic::atomic_bool;

}

#endif // ROSATOMIC_ATOMIC_H
