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

#include "ringbuffer.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/*  内部辅助函数                                                        */
/* ------------------------------------------------------------------ */

/**
 * @brief  判断环形缓冲区当前状态（内部使用）
 *
 * 利用镜像位区分满/空：
 *   - 读写索引相等且镜像位相同 → 空
 *   - 读写索引相等且镜像位不同 → 满
 *   - 其余情况                 → 半满
 */
static rt_inline enum rt_ringbuffer_state rt_ringbuffer_status(struct rt_ringbuffer *rb)
{
    if (rb->read_index == rb->write_index)
    {
        if (rb->read_mirror == rb->write_mirror)
            return RT_RINGBUFFER_EMPTY;
        else
            return RT_RINGBUFFER_FULL;
    }
    return RT_RINGBUFFER_HALFFULL;
}

/* ------------------------------------------------------------------ */
/*  接口函数实现                                                        */
/* ------------------------------------------------------------------ */

void rt_ringbuffer_init(struct rt_ringbuffer *rb,
                        rt_uint8_t           *pool,
                        rt_int16_t            size)
{
    RT_ASSERT(rb != NULL);
    RT_ASSERT(size > 0);

    /* 读写指针和镜像位全部归零 */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    /* 绑定内存池，容量向下对齐到 4 字节 */
    rb->buffer_ptr = pool;
    rb->buffer_size = RT_ALIGN_DOWN(size, 4);
}

void rt_ringbuffer_reset(struct rt_ringbuffer *rb)
{
    RT_ASSERT(rb != RT_NULL);

    rb->read_mirror  = 0;
    rb->read_index   = 0;
    rb->write_mirror = 0;
    rb->write_index  = 0;
}

rt_size_t rt_ringbuffer_put(struct rt_ringbuffer *rb,
                            const rt_uint8_t     *ptr,
                            rt_uint16_t           length)
{
    rt_uint16_t space;

    RT_ASSERT(rb != RT_NULL);

    space = rt_ringbuffer_space_len(rb);
    if (space == 0)
        return 0;

    /* 可用空间不足时，截断写入长度 */
    if (space < length)
        length = space;

    if (rb->buffer_size - rb->write_index > length)
    {
        /* 写指针到缓冲区末尾的连续空间足够，直接拷贝 */
        rt_memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        rb->write_index += length;
        return length;
    }

    /* 写入跨越缓冲区末尾，分两段拷贝 */
    rt_memcpy(&rb->buffer_ptr[rb->write_index],
              &ptr[0],
              rb->buffer_size - rb->write_index);
    rt_memcpy(&rb->buffer_ptr[0],
              &ptr[rb->buffer_size - rb->write_index],
              length - (rb->buffer_size - rb->write_index));

    /* 写指针越过末尾，翻转镜像位并回绕到头部 */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index  = length - (rb->buffer_size - rb->write_index);

    return length;
}

rt_size_t rt_ringbuffer_put_force(struct rt_ringbuffer *rb,
                                  const rt_uint8_t     *ptr,
                                  rt_uint16_t           length)
{
    rt_uint16_t space;

    RT_ASSERT(rb != RT_NULL);

    space = rt_ringbuffer_space_len(rb);

    /* 写入量超过缓冲区总容量时，只保留最新的 buffer_size 字节 */
    if (length > rb->buffer_size)
    {
        ptr    = &ptr[length - rb->buffer_size];
        length = rb->buffer_size;
    }

    if (rb->buffer_size - rb->write_index > length)
    {
        /* 连续空间足够，直接拷贝 */
        rt_memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        rb->write_index += length;

        /* 若覆盖了旧数据，读指针跟随写指针 */
        if (length > space)
            rb->read_index = rb->write_index;

        return length;
    }

    /* 分两段拷贝 */
    rt_memcpy(&rb->buffer_ptr[rb->write_index],
              &ptr[0],
              rb->buffer_size - rb->write_index);
    rt_memcpy(&rb->buffer_ptr[0],
              &ptr[rb->buffer_size - rb->write_index],
              length - (rb->buffer_size - rb->write_index));

    /* 写指针越过末尾，翻转镜像位并回绕 */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index  = length - (rb->buffer_size - rb->write_index);

    if (length > space)
    {
        /* 覆盖了旧数据，读指针同步跟随写指针 */
        if (rb->write_index <= rb->read_index)
            rb->read_mirror = ~rb->read_mirror;
        rb->read_index = rb->write_index;
    }

    return length;
}

rt_size_t rt_ringbuffer_get(struct rt_ringbuffer *rb,
                            rt_uint8_t           *ptr,
                            rt_uint16_t           length)
{
    rt_size_t avail;

    RT_ASSERT(rb != RT_NULL);

    avail = rt_ringbuffer_data_len(rb);
    if (avail == 0)
        return 0;

    /* 可读数据不足时，截断读取长度 */
    if (avail < length)
        length = avail;

    if (rb->buffer_size - rb->read_index > length)
    {
        /* 读指针到末尾的连续数据足够，直接拷贝 */
        rt_memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        rb->read_index += length;
        return length;
    }

    /* 读取跨越缓冲区末尾，分两段拷贝 */
    rt_memcpy(&ptr[0],
              &rb->buffer_ptr[rb->read_index],
              rb->buffer_size - rb->read_index);
    rt_memcpy(&ptr[rb->buffer_size - rb->read_index],
              &rb->buffer_ptr[0],
              length - (rb->buffer_size - rb->read_index));

    /* 读指针越过末尾，翻转镜像位并回绕 */
    rb->read_mirror = ~rb->read_mirror;
    rb->read_index  = length - (rb->buffer_size - rb->read_index);

    return length;
}

rt_size_t rt_ringbuffer_peek(struct rt_ringbuffer *rb, rt_uint8_t **ptr)
{
    rt_size_t avail;

    RT_ASSERT(rb != RT_NULL);

    *ptr = RT_NULL;

    avail = rt_ringbuffer_data_len(rb);
    if (avail == 0)
        return 0;

    /* 返回当前读指针地址 */
    *ptr = &rb->buffer_ptr[rb->read_index];

    if ((rt_size_t)(rb->buffer_size - rb->read_index) > avail)
    {
        /* 数据在物理上连续，读指针直接推进 */
        rb->read_index += avail;
        return avail;
    }

    /* 数据跨越末尾，只返回到末尾的连续部分 */
    avail = rb->buffer_size - rb->read_index;

    /* 读指针越过末尾，翻转镜像位并回绕到头部 */
    rb->read_mirror = ~rb->read_mirror;
    rb->read_index  = 0;

    return avail;
}

rt_size_t rt_ringbuffer_putchar(struct rt_ringbuffer *rb, const rt_uint8_t ch)
{
    RT_ASSERT(rb != RT_NULL);

    if (!rt_ringbuffer_space_len(rb))
        return 0;

    rb->buffer_ptr[rb->write_index] = ch;

    /* 写指针到达末尾时翻转镜像位并回绕 */
    if (rb->write_index == rb->buffer_size - 1)
    {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index  = 0;
    }
    else
    {
        rb->write_index++;
    }

    return 1;
}

rt_size_t rt_ringbuffer_putchar_force(struct rt_ringbuffer *rb, const rt_uint8_t ch)
{
    enum rt_ringbuffer_state old_state;

    RT_ASSERT(rb != RT_NULL);

    old_state = rt_ringbuffer_status(rb);

    rb->buffer_ptr[rb->write_index] = ch;

    if (rb->write_index == rb->buffer_size - 1)
    {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index  = 0;
        /* 若原本已满，读指针同步跟随写指针（覆盖最旧数据） */
        if (old_state == RT_RINGBUFFER_FULL)
        {
            rb->read_mirror = ~rb->read_mirror;
            rb->read_index  = rb->write_index;
        }
    }
    else
    {
        rb->write_index++;
        if (old_state == RT_RINGBUFFER_FULL)
            rb->read_index = rb->write_index;
    }

    return 1;
}

rt_size_t rt_ringbuffer_getchar(struct rt_ringbuffer *rb, rt_uint8_t *ch)
{
    RT_ASSERT(rb != RT_NULL);

    if (!rt_ringbuffer_data_len(rb))
        return 0;

    *ch = rb->buffer_ptr[rb->read_index];

    /* 读指针到达末尾时翻转镜像位并回绕 */
    if (rb->read_index == rb->buffer_size - 1)
    {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index  = 0;
    }
    else
    {
        rb->read_index++;
    }

    return 1;
}

rt_size_t rt_ringbuffer_data_len(struct rt_ringbuffer *rb)
{
    switch (rt_ringbuffer_status(rb))
    {
    case RT_RINGBUFFER_EMPTY:
        return 0;
    case RT_RINGBUFFER_FULL:
        return rb->buffer_size;
    case RT_RINGBUFFER_HALFFULL:
    default:
    {
        rt_size_t wi = rb->write_index;
        rt_size_t ri = rb->read_index;
        /* 写指针在读指针前方：直接相减；否则绕一圈计算 */
        return (wi > ri) ? (wi - ri) : (rb->buffer_size - (ri - wi));
    }
    }
}

#ifdef RT_USING_HEAP

rt_size_t rt_ringbuffer_create(rt_uint16_t size)
{
    struct rt_ringbuffer *rb;
    rt_uint8_t *pool;

    RT_ASSERT(size > 0);

    size = RT_ALIGN_DOWN(size, RT_ALIGN_SIZE);

    rb = (struct rt_ringbuffer *)rt_malloc(sizeof(struct rt_ringbuffer));
    if (rb == RT_NULL)
        goto exit;

    pool = (rt_uint8_t *)rt_malloc(size);
    if (pool == RT_NULL)
    {
        rt_free(rb);
        rb = RT_NULL;
        goto exit;
    }
    rt_ringbuffer_init(rb, pool, size);

exit:
    return rb;
}

void rt_ringbuffer_destroy(struct rt_ringbuffer *rb)
{
    RT_ASSERT(rb != RT_NULL);

    rt_free(rb->buffer_ptr);
    rt_free(rb);
}

#endif /* RT_USING_HEAP */