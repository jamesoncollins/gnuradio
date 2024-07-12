/* -*- c++ -*- */
/*
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>

 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "device_source_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/thread/thread.h>

#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace gr {
namespace iio {

template <class T>
typename device_source<T>::sptr device_source<T>::make(const std::string& uri,
                                        const std::string& device,
                                        const std::vector<std::string>& channels,
                                        const std::string& device_phy,
                                        const iio_param_vec_t& params,
                                        unsigned int buffer_size,
                                        unsigned int decimation)
{
    return gnuradio::make_block_sptr<device_source_impl<T>>(
        device_source_impl<T>::get_context(uri),
        true,
        device,
        channels,
        device_phy,
        params,
        buffer_size,
        decimation);
}


template <class T>
typename device_source<T>::sptr device_source<T>::make_from(iio_context* ctx,
                                             const std::string& device,
                                             const std::vector<std::string>& channels,
                                             const std::string& device_phy,
                                             const iio_param_vec_t& params,
                                             unsigned int buffer_size,
                                             unsigned int decimation)
{
    return gnuradio::make_block_sptr<device_source_impl<T>>(
        ctx, false, device, channels, device_phy, params, buffer_size, decimation);
}


template <class T>
void device_source_impl<T>::set_params(iio_device* phy, const iio_param_vec_t& params)
{
    static gr::logger logger("device_source_impl::set_params");

    for (auto& param : params) {
        iio_channel* chn = NULL;
        const char* attr = NULL;
        int ret;

        std::string key = param.first;
        std::string val = param.second;

        ret = iio_device_identify_filename(phy, key.c_str(), &chn, &attr);
        if (ret) {
            logger.warn("set_params: Parameter not recognized: {}", key);
            continue;
        }

        if (chn)
            ret = iio_channel_attr_write(chn, attr, val.c_str());
        else if (iio_device_find_attr(phy, attr))
            ret = iio_device_attr_write(phy, attr, val.c_str());
        else
            ret = iio_device_debug_attr_write(phy, attr, val.c_str());
        if (ret < 0) {
            logger.warn(
                "set_params: Unable to write attribute {:s}: {:d} {:s}", key, ret, val);
        }
    }
}

template <class T>
void device_source_impl<T>::set_params(const iio_param_vec_t& params)
{
    set_params(this->phy, params);
}

template <class T>
void device_source_impl<T>::set_len_tag_key(const std::string& len_tag_key)
{
    if (!len_tag_key.size()) {
        d_len_tag_key = pmt::PMT_NIL;
    } else {
        d_len_tag_key = pmt::string_to_symbol(len_tag_key);
    }
}

template <class T>
void device_source_impl<T>::set_buffer_size(unsigned int _buffer_size)
{
    std::unique_lock<std::mutex> lock(iio_mutex);

    if (buf && this->buffer_size != _buffer_size) {
        iio_buffer_destroy(buf);

        buf = iio_device_create_buffer(dev, _buffer_size, false);
        if (!buf)
            throw std::runtime_error("Unable to create buffer!\n");
    }

    this->buffer_size = _buffer_size;
}

template <class T>
void device_source_impl<T>::set_timeout_ms(unsigned long _timeout)
{
    this->timeout = _timeout;
}

template <class T>
iio_context* device_source_impl<T>::get_context(const std::string& uri)
{
    iio_context* ctx;

    // Check if we have a context with the same URI open
    if (!contexts.empty()) {
        for (ctx_it it = contexts.begin(); it != contexts.end(); ++it) {
            if (it->uri.compare(uri) == 0) {
                it->count++;
                return it->ctx;
            }
        }
    }

    if (uri.empty()) {
        ctx = iio_create_default_context();
        if (!ctx)
            ctx = iio_create_network_context(NULL);
    } else {
        ctx = iio_create_context_from_uri(uri.c_str());

        /* Stay compatible with old graphs, by accepting an
         * IP/hostname instead of an URI */
        if (!ctx)
            ctx = iio_create_network_context(uri.c_str());
    }
    // Save context info for future checks
    ctxInfo ci = { uri, ctx, 1 };
    contexts.push_back(ci);

    return ctx;
}


/*
 * The private constructor
 */
template <class T>
device_source_impl<T>::device_source_impl(iio_context* ctx,
                                       bool destroy_ctx,
                                       const std::string& device,
                                       const std::vector<std::string>& channels,
                                       const std::string& device_phy,
                                       const iio_param_vec_t& params,
                                       unsigned int buffer_size,
                                       unsigned int decimation)
    : gr::sync_block("device_source",
                     gr::io_signature::make(0, 0, 0),
                     gr::io_signature::make(1, -1, sizeof(T))),
      port_id(pmt::mp("msg")),
      timeout(100),
      d_len_tag_key(pmt::PMT_NIL),
      ctx(ctx),
      buf(NULL),
      buffer_size(buffer_size),
      decimation(decimation),
      destroy_ctx(destroy_ctx),
      thread_stopped(false)
{
    unsigned int nb_channels, i;

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
            iio_channel* chn = iio_device_find_channel(dev, it->c_str(), false);
            if (!chn) {
                if (destroy_ctx)
                    iio_context_destroy(ctx);
                throw std::runtime_error("Channel not found");
            }

            iio_channel_enable(chn);
            channel_list.push_back(chn);
        }
    }

    set_params(params);
    
    this->set_output_multiple(0x400);
    this->message_port_register_out(port_id);
}

template <class T>
void device_source_impl<T>::remove_ctx_history(iio_context* ctx_from_block, bool destroy_ctx)
{
    std::lock_guard<std::mutex> lock(ctx_mutex);

    for (ctx_it it = contexts.begin(); it != contexts.end(); ++it) {
        if (it->ctx == ctx_from_block) {
            if (it->count == 1) {
                if (destroy_ctx)
                    iio_context_destroy(ctx_from_block);
                it = contexts.erase(it);
                return;
            } else
                it->count--;
        }
    }
}

/*
 * Our virtual destructor.
 */
template <class T>
device_source_impl<T>::~device_source_impl()
{
    // Make sure this is the last open block with a given context
    // before removing the context
    remove_ctx_history(ctx, destroy_ctx);
}

template <class T>
float device_source_impl<T>::cast_data_type_byte(long tmpbuf)
{
    return (float)(*(int8_t *)&tmpbuf);
}

template <class T>
float device_source_impl<T>::cast_data_type_short(long tmpbuf)
{
    return (float)(*(int16_t *)&tmpbuf);
}

template <class T>
float device_source_impl<T>::cast_data_type_int(long tmpbuf)
{
    return (float)(*(int32_t *)&tmpbuf);
}

template <>
void device_source_impl<float>::channel_read(const iio_channel* chn, float* dst, size_t len)
{
    uintptr_t src_ptr;
    unsigned int length = (iio_channel_get_data_format(chn)->length + 7) / 8;
    uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
    ptrdiff_t buf_step = iio_buffer_step(buf) * (decimation + 1);
    bool is_signed = iio_channel_get_data_format(chn)->is_signed;    

    float (device_source_impl<float>::*float_cast)(long); 

    if (length == 1) 
        float_cast = &device_source_impl<float>::cast_data_type_byte;
    else if (length == 2) 
        float_cast = &device_source_impl<float>::cast_data_type_short;
    else if (length == 4)
        float_cast = &device_source_impl<float>::cast_data_type_int;
    else
        throw std::runtime_error("Sample size greater than 32 bits!\n");

    long tmpbuf;
    long max_val = (1 << ((length * 8) - 1)) - 1; // max signed value for underlying size

    size_t i = 0;
    for (src_ptr = (uintptr_t)iio_buffer_first(buf, chn) + byte_offset;
         src_ptr < buf_end && i < len;
         src_ptr += buf_step, i++) {

        iio_channel_convert(chn, (void*)&tmpbuf, (const void*)src_ptr);

        dst[i] = ((this->*float_cast)(tmpbuf)) / max_val;
    }
}

template <class T>
void device_source_impl<T>::channel_read(const iio_channel* chn, T* dst, size_t len)
{
    uintptr_t src_ptr;
    unsigned int length = (iio_channel_get_data_format(chn)->length + 7) / 8;
    uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);
    ptrdiff_t buf_step = iio_buffer_step(buf) * (decimation + 1);

    // additional checks are needed if not automatically converted to float
    if (length != sizeof(T))
        throw std::runtime_error("Sample size doesn't match chosen output type!\n");

    T tmpbuf;
    size_t i = 0;
    for (src_ptr = (uintptr_t)iio_buffer_first(buf, chn) + byte_offset;
         src_ptr < buf_end && i < len;
         src_ptr += buf_step, i++) {
        iio_channel_convert(chn, (void*)&dst[i], (const void*)src_ptr);
    }
}

template <class T>
int device_source_impl<T>::work(int noutput_items,
                             gr_vector_const_void_star& input_items,
                             gr_vector_void_star& output_items)
{
    ssize_t ret;

    // Check if we've processed what we have first
    if (!items_in_buffer) {
        ret = iio_buffer_refill(buf);
        if (ret < 0) {
            /* -EBADF happens when the buffer is cancelled */
            if (ret != -EBADF) {

                char buf[256];
                iio_strerror(-ret, buf, sizeof(buf));
                this->d_logger->warn("Unable to refill buffer: {:s}", buf);
            }
            return -1;
        }

        items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);
        if (!items_in_buffer)
            return 0;

        byte_offset = 0;

        // Tag start of new packet
        if (d_len_tag_key != pmt::PMT_NIL) {
            size_t toutputs;
            if (override_tagged_output_channels > 0)
                toutputs = override_tagged_output_channels;
            else
                toutputs = output_items.size();

            for (size_t i = 0; i < toutputs; i += 1) {
                this->add_item_tag(i,
                                   this->nitems_written(0),
                                   this->d_len_tag_key,
                                   pmt::from_long(items_in_buffer));
            }
        }
    }

    // Process samples
    unsigned long items = std::min(items_in_buffer, (unsigned long)noutput_items);

    for (size_t i = 0; i < output_items.size(); i++)
        channel_read(channel_list[i], (T *)output_items[i], items);

    items_in_buffer -= items;
    byte_offset += items * iio_buffer_step(buf);

    return (int)items;
}

template <class T>
bool device_source_impl<T>::start()
{
    items_in_buffer = 0;
    byte_offset = 0;
    thread_stopped = false;

    buf = iio_device_create_buffer(dev, buffer_size, false);
    if (!buf) {
        throw std::runtime_error("Unable to create buffer!\n");
    }

    return !!buf;
}

template <class T>
bool device_source_impl<T>::stop()
{
    thread_stopped = true;

    if (buf)
        iio_buffer_cancel(buf);

    if (buf) {
        iio_buffer_destroy(buf);
        buf = NULL;
    }

    return true;
}

template <class T>
bool device_source_impl<T>::load_fir_filter(std::string& filter, iio_device* phy)
{
    if (filter.empty() || !iio_device_find_attr(phy, "filter_fir_config"))
        return false;

    std::ifstream ifs(filter.c_str(), std::ifstream::binary);
    if (!ifs)
        return false;

    /* Here, we verify that the filter file contains data for both RX+TX. */
    {
        char buf[256];

        do {
            ifs.getline(buf, sizeof(buf));
        } while (!(buf[0] == '-' || (buf[0] >= '0' && buf[0] <= '9')));

        std::string line(buf);
        if (line.find(',') == std::string::npos)
            throw std::runtime_error("Incompatible filter file");
    }

    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);

    char* buffer = new char[length];

    ifs.read(buffer, length);
    ifs.close();

    int ret = iio_device_attr_write_raw(phy, "filter_fir_config", buffer, length);

    delete[] buffer;
    return ret > 0;
}

template <class T>
int device_source_impl<T>::handle_decimation_interpolation(unsigned long samplerate,
                                                        const char* channel_name,
                                                        const char* attr_name,
                                                        iio_device* dev,
                                                        bool disable_dec,
                                                        bool output_chan)
{
    static gr::logger log("device_source_impl::handle_decimation_interpolation");
    int ret;
    iio_channel* chan;
    char buff[128];
    unsigned long long min, max;

    std::string an(attr_name);
    an.append("_available");

    // Get ranges
    chan = iio_device_find_channel(dev, channel_name, output_chan);
    if (chan == NULL)
        return -1; // Channel doesn't exist so the dec/int filters probably don't exist

    ret = iio_channel_attr_read(chan, an.c_str(), buff, sizeof(buff));
    if (ret < 0)
        return -1; // Channel attribute does not exist so no dec/int filter exist

    sscanf(buff, "%llu %llu ", &max, &min);

    // Write lower range (maybe)
    if (disable_dec)
        min = max;

    ret = iio_channel_attr_write_longlong(chan, "sampling_frequency", min);
    if (ret < 0) {
        log.warn("Unable to write attribute sampling_frequency!");
    }

    return ret;
}

template class device_source<std::int8_t>;
template class device_source_impl<std::int8_t>;

template class device_source<std::int16_t>;
template class device_source_impl<std::int16_t>;

template class device_source<std::int32_t>;
template class device_source_impl<std::int32_t>;

template class device_source<float>;
template class device_source_impl<float>;

template class device_source<std::complex<std::int16_t>>;
template class device_source_impl<std::complex<std::int16_t>>;
template class device_source<gr_complex>;
template class device_source_impl<gr_complex>;
// ^^
// needed for compatibility with fmcomms2



} /* namespace iio */
} /* namespace gr */
