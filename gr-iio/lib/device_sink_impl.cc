/* -*- c++ -*- */
/*
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "device_sink_impl.h"
#include "device_source_impl.h"
#include <gnuradio/io_signature.h>

#include <string>
#include <vector>

namespace gr {
namespace iio {

template <class T>
typename device_sink<T>::sptr device_sink<T>::make(const std::string& uri,
                                    const std::string& device,
                                    const std::vector<std::string>& channels,
                                    const std::string& device_phy,
                                    const iio_param_vec_t& params,
                                    unsigned int buffer_size,
                                    unsigned int interpolation,
                                    bool cyclic)
{
    return gnuradio::make_block_sptr<device_sink_impl<T>>(
        device_source_impl<T>::get_context(uri),
        true,
        device,
        channels,
        device_phy,
        params,
        buffer_size,
        interpolation,
        cyclic);
}

template <class T>
typename device_sink<T>::sptr device_sink<T>::make_from(iio_context* ctx,
                                         const std::string& device,
                                         const std::vector<std::string>& channels,
                                         const std::string& device_phy,
                                         const iio_param_vec_t& params,
                                         unsigned int buffer_size,
                                         unsigned int interpolation,
                                         bool cyclic)
{
    return gnuradio::make_block_sptr<device_sink_impl<T>>(ctx,
                                                       false,
                                                       device,
                                                       channels,
                                                       device_phy,
                                                       params,
                                                       buffer_size,
                                                       interpolation,
                                                       cyclic);
}

template <class T>
void device_sink_impl<T>::set_params(const iio_param_vec_t& params)
{
    device_source_impl<T>::set_params(this->phy, params);
}

/*
 * The private constructor
 */
template <class T>
device_sink_impl<T>::device_sink_impl(iio_context* ctx,
                                   bool destroy_ctx,
                                   const std::string& device,
                                   const std::vector<std::string>& channels,
                                   const std::string& device_phy,
                                   const iio_param_vec_t& params,
                                   unsigned int buffer_size,
                                   unsigned int interpolation,
                                   bool cyclic)
    : gr::sync_block("device_sink",
                     gr::io_signature::make(1, -1, sizeof(T)),
                     gr::io_signature::make(0, 0, 0)),
      d_tags(0),
      ctx(ctx),
      interpolation(interpolation),
      buffer_size(buffer_size),
      destroy_ctx(destroy_ctx),
      d_len_tag_key(pmt::PMT_NIL)
{
    unsigned int nb_channels, i;

    /* Set minimum input size */
    this->set_output_multiple(buffer_size / (interpolation + 1));

    if (!ctx)
        throw std::runtime_error("Unable to create context");

    dev = iio_context_find_device(ctx, device.c_str());
    phy = iio_context_find_device(ctx, device_phy.c_str());
    if (!dev || !phy) {
        if (destroy_ctx)
            iio_context_destroy(ctx);
        throw std::runtime_error("Device not found");
    }

    /* First disable all channels */
    nb_channels = iio_device_get_channels_count(dev);
    for (i = 0; i < nb_channels; i++)
        iio_channel_disable(iio_device_get_channel(dev, i));

    if (channels.empty()) {
        for (i = 0; i < nb_channels; i++) {
            iio_channel* chn = iio_device_get_channel(dev, i);

            iio_channel_enable(chn);
            channel_list.push_back(chn);
        }
    } else {
        for (std::vector<std::string>::const_iterator it = channels.begin();
             it != channels.end();
             ++it) {
            iio_channel* chn = iio_device_find_channel(dev, it->c_str(), true);
            if (!chn) {
                if (destroy_ctx)
                    iio_context_destroy(ctx);
                throw std::runtime_error("Channel not found");
            }

            iio_channel_enable(chn);
            if (!iio_channel_is_enabled(chn))
                throw std::runtime_error("Channel not enabled");
            channel_list.push_back(chn);
        }
    }

    set_params(params);

    buf = iio_device_create_buffer(dev, buffer_size, cyclic);
    if (!buf)
        throw std::runtime_error("Unable to create buffer: " + std::to_string(-errno));
}

/*
 * Our virtual destructor.
 */
template <class T>
device_sink_impl<T>::~device_sink_impl()
{
    iio_buffer_destroy(buf);
    device_source_impl<T>::remove_ctx_history(ctx, destroy_ctx);
}

template <>
void device_sink_impl<float>::channel_write(const iio_channel* chn, const float* src, size_t len)
{
    uintptr_t dst_ptr;
    unsigned int length = (iio_channel_get_data_format(chn)->length + 7) / 8;
    uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
    ptrdiff_t buf_step = iio_buffer_step(buf) * (interpolation + 1);

    long tmpbuf;
    long max_val = (1 << ((length * 8) - 1)) - 1; // max signed value for underlying size

    size_t i = 0;
    for (dst_ptr = (uintptr_t)iio_buffer_first(buf, chn);
         dst_ptr < buf_end && i < len;
         dst_ptr += buf_step, i++) {
        
        tmpbuf = (long)(src[i] * max_val);
        iio_channel_convert_inverse(chn, (void*)dst_ptr, (const void*)&tmpbuf);
    }
}

template <class T>
void device_sink_impl<T>::channel_write(const iio_channel* chn, const T* src, size_t len)
{
    uintptr_t dst_ptr;
    unsigned int length = (iio_channel_get_data_format(chn)->length + 7) / 8;
    uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
    ptrdiff_t buf_step = iio_buffer_step(buf) * (interpolation + 1);

    // additional checks are needed if not automatically converted to float
    if (length != sizeof(T))
        throw std::runtime_error("Sample size doesn't match chosen output type!\n");

    size_t i = 0;
    for (dst_ptr = (uintptr_t)iio_buffer_first(buf, chn);
         dst_ptr < buf_end && i < len;
         dst_ptr += buf_step, i++) {
        iio_channel_convert_inverse(chn, (void*)dst_ptr, (const void*)&src[i]);
    }
}

template <class T>
void device_sink_impl<T>::set_len_tag_key(const std::string& len_tag_key)
{
    if (!len_tag_key.size()) {
        d_len_tag_key = pmt::PMT_NIL;
    } else {
        d_len_tag_key = pmt::string_to_symbol(len_tag_key);
    }
}

template <class T>
int device_sink_impl<T>::work(int noutput_items,
                           gr_vector_const_void_star& input_items,
                           gr_vector_void_star& output_items)
{
    int ret;

    if (d_len_tag_key != pmt::PMT_NIL) {

        size_t ninputs;
        if (override_tagged_input_channels > 0)
            ninputs = override_tagged_input_channels;
        else
            ninputs = input_items.size();

        for (size_t i = 0; i < ninputs; i++) {
            auto items_read = this->nitems_read(i);
            this->get_tags_in_range(d_tags, i, items_read, items_read + 1, d_len_tag_key);

            if (d_tags.size() < 1) {
                throw std::runtime_error("device_sink: Input stream not tagged! Either "
                                         "disable tagged input or tag your stream!");
            }

            long required_size = buffer_size / (interpolation + 1);
            for (auto& tag : d_tags) {
                auto packet_len = pmt::to_long(tag.value);
                if (packet_len != required_size) {
                    throw std::runtime_error("Invalid packet size, expected " +
                                             std::to_string(required_size) + " and got " +
                                             std::to_string(packet_len) + "!");
                }
            }
        }
        // Everything is ok, we can proceed
    }

    if (interpolation >= 1) {
        ptrdiff_t len = (intptr_t)iio_buffer_end(buf) - (intptr_t)iio_buffer_start(buf);
        memset(iio_buffer_start(buf), 0, len);
    }

    for (unsigned int i = 0; i < input_items.size(); i++)
        channel_write(channel_list[i], (T *)input_items[i], noutput_items);

    ret = iio_buffer_push(buf);
    if (ret < 0) {
        char buf[256];
        iio_strerror(-ret, buf, sizeof(buf));
        std::string error(buf);

        this->d_logger->warn("Unable to push buffer: {:s}", error);
        return this->WORK_DONE; /* EOF */ // where does this come from???
    }

    this->consume_each(buffer_size / (interpolation + 1));
    return 0;
}

template <class T>
void device_sink_impl<T>::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    for (unsigned int i = 0; i < ninput_items_required.size(); i++)
        ninput_items_required[i] = noutput_items;
}

template class device_sink<std::int8_t>;
template class device_sink_impl<std::int8_t>;

template class device_sink<std::int16_t>;
template class device_sink_impl<std::int16_t>;

template class device_sink<std::int32_t>;
template class device_sink_impl<std::int32_t>;

template class device_sink<float>;
template class device_sink_impl<float>;

template class device_sink<std::complex<std::int16_t>>;
template class device_sink_impl<std::complex<std::int16_t>>;
template class device_sink<gr_complex>;
template class device_sink_impl<gr_complex>;

} /* namespace iio */
} /* namespace gr */
