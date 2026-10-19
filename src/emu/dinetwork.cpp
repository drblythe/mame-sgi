// license:BSD-3-Clause
// copyright-holders:Carl, Miodrag Milanovic

#include "emu.h"
#include "dinetwork.h"

#include "osdepend.h"

#include <algorithm>


device_network_interface::device_network_interface(const machine_config &mconfig, device_t &device, u32 bandwidth, u32 mtu)
	: device_interface(device, "network")
	, m_poll_timer(nullptr)
	, m_send_timer(nullptr)
	, m_recv_timer(nullptr)
	, m_retry_timer(nullptr)
{
	// Convert to Mibps to Bps
	m_bandwidth = bandwidth << (20 - 3);
	m_mtu = mtu;
	m_intf = -1;
	m_loopback_control = false;
	m_unthrottled = false;
	m_recv_retry = false;
}

device_network_interface::~device_network_interface()
{
}

void device_network_interface::interface_post_start()
{
	m_poll_timer = device().machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(device_network_interface::poll_device), this));
	m_send_timer = device().machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(device_network_interface::send_complete), this));
	m_recv_timer = device().machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(device_network_interface::recv_complete), this));
	if (m_recv_retry)
		m_retry_timer = device().machine().scheduler().timer_alloc(timer_expired_delegate(FUNC(device_network_interface::recv_retry), this));

	device().save_item(NAME(m_loopback_control));
}

void device_network_interface::interface_post_load()
{
	if (!m_dev)
		m_poll_timer->reset();
	else if (!m_loopback_control && !m_recv_timer->enabled())
		start_net_device();
	else
		stop_net_device();
}

attotime device_network_interface::transfer_delay(int len) const
{
	if (unthrottled())
		return attotime::zero;
	return attotime::from_ticks(len, m_bandwidth);
}

int device_network_interface::send(u8 *buf, int len, int fcs)
{
	// TODO: enable this check when other devices implement delayed transmit
	//if (m_send_timer->enabled())
		//throw emu_fatalerror("%s(%s): attempted to transmit while transmit already in progress", device().shortname(), device().tag());

	int result = 0;

	if (m_loopback_control)
	{
		// loop data back to receiver
		result = recv_start_cb(buf, len);

		if (result)
		{
			// schedule receive complete callback
			m_recv_timer->adjust(transfer_delay(len), result);
		}
	}
	else if (m_dev)
	{
		// send the data (excluding fcs)
		result = m_dev->send(buf, len - fcs);
		if (result)
			result += fcs;
	}

	// schedule transmit complete callback
	m_send_timer->adjust(transfer_delay(len), result);

	return result;
}

TIMER_CALLBACK_MEMBER(device_network_interface::poll_device)
{
	m_dev->poll();
}

void device_network_interface::start_net_device()
{
	// Set device polling time to transfer time for one MTU
	m_dev->start();
	const attotime interval = attotime::from_hz(m_bandwidth / m_mtu);
	m_poll_timer->adjust(attotime::zero, 0, interval);
}

void device_network_interface::stop_net_device()
{
	m_poll_timer->reset();
	m_dev->stop();
}

TIMER_CALLBACK_MEMBER(device_network_interface::send_complete)
{
	send_complete_cb(param);
}

void device_network_interface::recv_cb(u8 *buf, int len)
{
	if (m_recv_timer->enabled())
		throw emu_fatalerror("%s(%s): attempted to receive while receive already in progress", device().shortname(), device().tag());

	int result = 0;

	// process the received data
	if (!m_loopback_control)
		result = recv_start_cb(buf, len);

	if (result)
	{
		// stop receiving more data from the network
		if (m_dev)
			stop_net_device();

		// schedule receive complete callback
		m_recv_timer->adjust(transfer_delay(len), result);
	}
	else if (m_retry_timer && !m_loopback_control && m_dev && unthrottled() && recv_busy())
	{
		// Without the bandwidth model, frames can arrive faster than the
		// guest can possibly service them (with zero transfer delay, a
		// whole burst arrives without the emulated cpu running at all).
		// Rather than discard a frame the device is momentarily unable to
		// accept, hold it, stop polling (leaving further frames queued in
		// the host network stack) and retry shortly.
		m_recv_pending.assign(buf, buf + len);
		stop_net_device();
		m_retry_timer->adjust(recv_retry_interval());
	}
}

attotime device_network_interface::recv_retry_interval() const
{
	return attotime::from_usec(20);
}

TIMER_CALLBACK_MEMBER(device_network_interface::recv_retry)
{
	if (m_recv_pending.empty())
		return;

	if (recv_busy() && !m_loopback_control)
	{
		m_retry_timer->adjust(recv_retry_interval());
		return;
	}

	std::vector<u8> frame;
	frame.swap(m_recv_pending);

	int const result = m_loopback_control ? 0 : recv_start_cb(frame.data(), frame.size());
	if (result)
		m_recv_timer->adjust(transfer_delay(frame.size()), result);
	else if (m_dev && !m_loopback_control)
		start_net_device();
}

TIMER_CALLBACK_MEMBER(device_network_interface::recv_complete)
{
	recv_complete_cb(param);

	// start receiving data from the network again
	if (m_dev && !m_loopback_control)
		start_net_device();
}

void device_network_interface::set_mac(const u8 *mac)
{
	std::copy_n(mac, std::size(m_mac), std::begin(m_mac));
}

void device_network_interface::set_interface(int id)
{
	if (m_dev)
		stop_net_device();

	m_dev = device().machine().osd().open_network_device(id, *this);
	if (m_dev)
	{
		if (!m_loopback_control)
			start_net_device();
	}
	else
	{
		device().logerror("Network interface %d not found\n", id);
		id = -1;
	}
	m_intf = id;
}

void device_network_interface::set_loopback(bool loopback)
{
	if (m_loopback_control == loopback)
		return;

	m_loopback_control = loopback;

	// a frame held back for a busy receiver is discarded
	m_recv_pending.clear();
	if (m_retry_timer)
		m_retry_timer->enable(false);

	if (m_dev)
	{
		if (loopback)
			stop_net_device();
		else if (!m_recv_timer->enabled())
			start_net_device();
	}
}

void device_network_interface::log_bytes(const u8 *buf, int len)
{
	static const char *const frame_fmt = "%02x %02x %02x %02x %02x %02x %02x %02x\n";

	for (int i = 0; i < len / 8; i++)
		device().logerror(frame_fmt,
			buf[i * 8 + 0], buf[i * 8 + 1], buf[i * 8 + 2], buf[i * 8 + 3],
			buf[i * 8 + 4], buf[i * 8 + 5], buf[i * 8 + 6], buf[i * 8 + 7]);

	if (int const tail = len % 8)
		device().logerror(&frame_fmt[tail * 5],
			buf[len - tail + 0], buf[len - tail + 1], buf[len - tail + 2], buf[len - tail + 3],
			buf[len - tail + 4], buf[len - tail + 5], buf[len - tail + 6], buf[len - tail + 7]);
}
