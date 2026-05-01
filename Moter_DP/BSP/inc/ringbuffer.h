/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * 变更记录：
 * 日期            作者          说明
 * 2012-09-30     Bernard      初始版本
 * 2013-05-08     Grissiom     重新实现（镜像位算法）
 * 2016-08-18     heyuanjie    新增接口
 * 2021-07-20     arminker     修复 rt_ringbuffer_put_force 中 write_index 越界 bug
 * 2021-08-14     Jackistang   补充函数接口注释
 */

#ifndef RINGBUFFER_H__
#define RINGBUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdio.h"
#include "assert.h"

/* ------------------------------------------------------------------ */
/*  类型别名（与 RT-Thread 保持兼容）                                   */
/* ------------------------------------------------------------------ */
typedef uint8_t     rt_uint8_t;
typedef uint16_t    rt_uint16_t;
typedef int16_t     rt_int16_t;
typedef size_t      rt_size_t;

/* ------------------------------------------------------------------ */
/*  宏定义                                                              */
/* ------------------------------------------------------------------ */
#define RT_ASSERT   assert
#define RT_NULL     NULL
#define rt_inline   inline
#define rt_memcpy   memcpy

/** 向下对齐到 align 的整数倍（align 必须是 2 的幂次方） */
#define RT_ALIGN_DOWN(size, align)  ((size) & ~((align) - 1))

/* ------------------------------------------------------------------ */
/*  核心数据结构                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief 环形缓冲区控制块
 *
 * 采用"镜像位"算法区分满/空状态，避免浪费一个字节：
 *   - read_index == write_index 且镜像位相同  → 缓冲区为空
 *   - read_index == write_index 且镜像位不同  → 缓冲区已满
 *
 * 示意图（容量 7 字节）：
 *
 *   mirror=0 侧                  mirror=1 侧（虚拟镜像）
 *   +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
 *   | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 |  满
 *   +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
 *    read_idx-^                   write_idx-^
 *
 *   +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
 *   | 0 | 1 | 2 | 3 | 4 | 5 | 6 ||| 0 | 1 | 2 | 3 | 4 | 5 | 6 |  空
 *   +---+---+---+---+---+---+---+|+~~~+~~~+~~~+~~~+~~~+~~~+~~~+
 *   read_idx-^ ^-write_idx
 *
 * @note 由于索引使用 15 位，单个缓冲区最大容量为 32767 字节。
 */
struct rt_ringbuffer
{
    rt_uint8_t  *buffer_ptr;            /**< 指向实际存储内存的指针              */

    rt_uint16_t  read_mirror  : 1;      /**< 读指针镜像位（用于满/空判断）       */
    rt_uint16_t  read_index   : 15;     /**< 读指针偏移（0 ~ buffer_size-1）     */
    rt_uint16_t  write_mirror : 1;      /**< 写指针镜像位（用于满/空判断）       */
    rt_uint16_t  write_index  : 15;     /**< 写指针偏移（0 ~ buffer_size-1）     */

    rt_int16_t   buffer_size;           /**< 缓冲区总容量（字节），必须为正数    */
};

/**
 * @brief 环形缓冲区状态枚举
 */
enum rt_ringbuffer_state
{
    RT_RINGBUFFER_EMPTY,    /**< 缓冲区为空（无可读数据）                        */
    RT_RINGBUFFER_FULL,     /**< 缓冲区已满（无可写空间）                        */
    RT_RINGBUFFER_HALFFULL, /**< 缓冲区半满（既有数据可读，也有空间可写）        */
};

/* ------------------------------------------------------------------ */
/*  接口函数声明                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  初始化环形缓冲区对象
 *
 * 将读写指针和镜像位全部清零，并绑定外部提供的内存池。
 * 实际可用容量会向下对齐到 4 字节边界。
 *
 * @param  rb    指向环形缓冲区控制块的指针
 * @param  pool  指向外部内存池的指针（调用方负责分配和释放）
 * @param  size  内存池大小（字节），实际容量 = RT_ALIGN_DOWN(size, 4)
 */
void rt_ringbuffer_init(struct rt_ringbuffer *rb, rt_uint8_t *pool, rt_int16_t size);

/**
 * @brief  重置环形缓冲区（清空所有数据，不释放内存）
 *
 * @param  rb  指向环形缓冲区控制块的指针
 */
void rt_ringbuffer_reset(struct rt_ringbuffer *rb);

/**
 * @brief  向缓冲区写入一块数据（空间不足时丢弃超出部分）
 *
 * @param  rb      指向环形缓冲区控制块的指针
 * @param  ptr     指向待写入数据的指针
 * @param  length  待写入字节数
 * @return 实际写入的字节数（可能小于 length）
 */
rt_size_t rt_ringbuffer_put(struct rt_ringbuffer *rb, const rt_uint8_t *ptr, rt_uint16_t length);

/**
 * @brief  向缓冲区强制写入一块数据（空间不足时覆盖最旧数据）
 *
 * @param  rb      指向环形缓冲区控制块的指针
 * @param  ptr     指向待写入数据的指针
 * @param  length  待写入字节数
 * @return 实际写入的字节数（始终等于 length 或 buffer_size 中的较小值）
 */
rt_size_t rt_ringbuffer_put_force(struct rt_ringbuffer *rb, const rt_uint8_t *ptr, rt_uint16_t length);

/**
 * @brief  向缓冲区写入单个字节（空间不足时写入失败）
 *
 * @param  rb  指向环形缓冲区控制块的指针
 * @param  ch  待写入的字节
 * @return 1 表示写入成功，0 表示缓冲区已满
 */
rt_size_t rt_ringbuffer_putchar(struct rt_ringbuffer *rb, const rt_uint8_t ch);

/**
 * @brief  向缓冲区强制写入单个字节（空间不足时覆盖最旧数据）
 *
 * @param  rb  指向环形缓冲区控制块的指针
 * @param  ch  待写入的字节
 * @return 始终返回 1
 */
rt_size_t rt_ringbuffer_putchar_force(struct rt_ringbuffer *rb, const rt_uint8_t ch);

/**
 * @brief  从缓冲区读取一块数据并移动读指针
 *
 * @param  rb      指向环形缓冲区控制块的指针
 * @param  ptr     指向接收缓冲区的指针
 * @param  length  期望读取的字节数
 * @return 实际读取的字节数（可能小于 length）
 */
rt_size_t rt_ringbuffer_get(struct rt_ringbuffer *rb, rt_uint8_t *ptr, rt_uint16_t length);

/**
 * @brief  窥视缓冲区中第一个可读字节的地址（同时移动读指针）
 *
 * 返回当前读指针所指向的地址，并将读指针推进到下一段连续区域的起点。
 * 由于环形缓冲区可能在物理上不连续，建议每次只读取 1 字节。
 *
 * @param  rb   指向环形缓冲区控制块的指针
 * @param  ptr  输出参数，*ptr 指向第一个可读字节
 * @return 本次可连续读取的字节数（到缓冲区末尾为止）
 */
rt_size_t rt_ringbuffer_peek(struct rt_ringbuffer *rb, rt_uint8_t **ptr);

/**
 * @brief  从缓冲区读取单个字节并移动读指针
 *
 * @param  rb  指向环形缓冲区控制块的指针
 * @param  ch  输出参数，存储读取到的字节
 * @return 1 表示读取成功，0 表示缓冲区为空
 */
rt_size_t rt_ringbuffer_getchar(struct rt_ringbuffer *rb, rt_uint8_t *ch);

/**
 * @brief  获取缓冲区中当前可读数据的字节数
 *
 * @param  rb  指向环形缓冲区控制块的指针
 * @return 可读字节数
 */
rt_size_t rt_ringbuffer_data_len(struct rt_ringbuffer *rb);

#ifdef RT_USING_HEAP
/**
 * @brief  动态创建环形缓冲区对象（需要堆支持）
 *
 * 内部调用 rt_malloc 分配控制块和内存池，使用完毕后
 * 必须调用 rt_ringbuffer_destroy 释放。
 *
 * @param  length  期望的缓冲区容量（字节）
 * @return 指向新建控制块的指针，失败时返回 RT_NULL
 */
struct rt_ringbuffer *rt_ringbuffer_create(rt_uint16_t length);

/**
 * @brief  销毁由 rt_ringbuffer_create 创建的环形缓冲区对象
 *
 * @param  rb  指向环形缓冲区控制块的指针
 */
void rt_ringbuffer_destroy(struct rt_ringbuffer *rb);
#endif

/* ------------------------------------------------------------------ */
/*  内联函数                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief  获取缓冲区总容量
 *
 * @param  rb  指向环形缓冲区控制块的指针
 * @return 缓冲区总容量（字节）
 */
rt_inline rt_uint16_t rt_ringbuffer_get_size(struct rt_ringbuffer *rb)
{
    RT_ASSERT(rb != RT_NULL);
    return rb->buffer_size;
}

/** 获取缓冲区剩余可写空间（字节） */
#define rt_ringbuffer_space_len(rb) ((rb)->buffer_size - rt_ringbuffer_data_len(rb))

#ifdef __cplusplus
}
#endif

#endif /* RINGBUFFER_H__ */