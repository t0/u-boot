// SPDX-License-Identifier: GPL-2.0+
/*
 * NETCONSOLE over the lwIP raw UDP API. Mirrors the legacy in-tree
 * driver at drivers/net/netconsole.c, but uses udp_new()/udp_bind()/
 * udp_recv()/udp_sendto() instead of net_loop()/net_send_udp_packet().
 *
 */

#include <command.h>
#include <env.h>
#include <log.h>
#include <stdio_dev.h>
#include <time.h>
#include <net.h>
#include <vsprintf.h>
#include <u-boot/schedule.h>
#include <lwip/ip_addr.h>
#include <lwip/ip4_addr.h>
#include <lwip/netif.h>
#include <lwip/pbuf.h>
#include <lwip/udp.h>
#include <lwip/etharp.h>

#ifndef CONFIG_NETCONSOLE_BUFFER_SIZE
#define CONFIG_NETCONSOLE_BUFFER_SIZE 512
#endif

static char input_buffer[CONFIG_NETCONSOLE_BUFFER_SIZE];
static int input_size; /* char count in input buffer */
static int input_offset; /* offset to valid chars in input buffer */
static int input_recursion;
static int output_recursion;
static bool nc_ip_bcast; /* server ip is a broadcast address */
static ip_addr_t nc_ip; /* server ip */
static u16_t nc_out_port; /* target output port */
static u16_t nc_in_port; /* source input port */

/* lwIP-specific state */
static struct udp_pcb *nc_pcb;
static struct netif *nc_netif;
static u16_t nc_bound_port;

static struct netif *nc_get_netif(void)
{
	struct udevice *udev;
	struct netif *current = net_lwip_get_netif();
	if (current)
		return current;

	if (net_lwip_eth_start() < 0)
		return NULL;

	udev = eth_get_dev();
	if (!udev)
		return NULL;

	return net_lwip_new_netif(udev);
}

static int is_broadcast(ip_addr_t ip)
{
	static ip_addr_t netmask;
	static ip_addr_t our_ip;
	static int env_changed_id;
	int env_id = env_get_id();
	u32_t nm, oi, addr;

	/* update only when the environment has changed */
	if (env_changed_id != env_id) {
		const char *p;

		ip4_addr_set_zero(ip_2_ip4(&netmask));
		ip4_addr_set_zero(ip_2_ip4(&our_ip));
		p = env_get("netmask");
		if (p)
			ipaddr_aton(p, &netmask);
		p = env_get("ipaddr");
		if (p)
			ipaddr_aton(p, &our_ip);

		env_changed_id = env_id;
	}

	nm = ip4_addr_get_u32(ip_2_ip4(&netmask));
	oi = ip4_addr_get_u32(ip_2_ip4(&our_ip));
	addr = ip4_addr_get_u32(ip_2_ip4(&ip));

	return (addr == ~0U || /* 255.255.255.255 (global bcast) */
		((nm & oi) ==
		 (nm & addr) && /* on the same net and */
		 (nm | addr) == ~0U)); /* bcast to our net */
}

static int refresh_settings_from_env(void)
{
	const char *p;
	static int env_changed_id;
	int env_id = env_get_id();

	/* update only when the environment has changed */
	if (env_changed_id != env_id) {
		char *tmp = env_get("ncip");
		if (tmp) {
			/* splits tmp into ip:port */
			const char *colon = strchr(tmp, ':');
			size_t n = colon ? (size_t)(colon - tmp) : strlen(tmp);
			char ipstr[16];
			if (n >= sizeof(ipstr))
				return -1;
			memcpy(ipstr, tmp, n);
			ipstr[n] = '\0';

			if (!ipaddr_aton(ipstr, &nc_ip))
				return -1; /* not a valid ip */
			if (ip4_addr_isany_val(*ip_2_ip4(&nc_ip)))
				return -1;	/* ncip is 0.0.0.0 */
			if (colon) {
				nc_out_port = dectoul(colon + 1, NULL);
				nc_in_port = nc_out_port;
			}
		} else {
			/* ncip is not set, so broadcast */
			ip_addr_set_ip4_u32(&nc_ip, IPADDR_BROADCAST);
		}

		p = env_get("ncoutport");
		if (p != NULL)
			nc_out_port = dectoul(p, NULL);
		p = env_get("ncinport");
		if (p != NULL)
			nc_in_port = dectoul(p, NULL);

		nc_ip_bcast = is_broadcast(nc_ip);
		env_changed_id = env_id;
	}
	return 0;
}

/*
 * Remove the pcb and netif manually
 */
static void nc_teardown(void)
{
	if (nc_pcb) {
		udp_remove(nc_pcb);
		nc_pcb = NULL;
		nc_bound_port = 0;
	}
	if (nc_netif) {
		if (net_lwip_get_netif() == nc_netif)
			netif_remove(nc_netif);
		free(nc_netif);
		nc_netif = NULL;
	}
}

/*
 * Netconsole output needs the client's MAC: check the ARP cache and, on
 * a miss, ask lwIP to resolve it. etharp_query() with a NULL pbuf
 * creates (or refreshes) the pending ARP entry and broadcasts one
 * request.
 *
 * Returns true if output can be sent now (client resolved, or broadcast).
 */
static bool nc_arp_resolve(void)
{
	static ulong last;
	/* Unused, but required: etharp_find_addr() writes through its
	 * out-parameters without checking them for NULL. */
	struct eth_addr *ethaddr;
	const ip4_addr_t *ipaddr;
	struct netif *netif;

	if (nc_ip_bcast)
		return true; /* broadcast needs no ARP */

	netif = nc_get_netif();
	if (!netif)
		return false;

	if (etharp_find_addr(netif, ip_2_ip4(&nc_ip),
			     &ethaddr, &ipaddr) >= 0)
		return true;

	if (!last || get_timer(last) >= ARP_TMR_INTERVAL) {
		etharp_query(netif, ip_2_ip4(&nc_ip), NULL);
		last = get_timer(0);
	}

	return false;
}

/* Copies a received datagram's payload into input_buffer */
static void nc_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,
			    const ip_addr_t *addr, u16_t port)
{
	int end, chunk, len;

	if (!p)
		return;

	if (!nc_ip_bcast && !ip_addr_cmp(addr, &nc_ip)) {
		pbuf_free(p);
		return; /* not from our client */
	}

	len = p->tot_len;
	if (!len) {
		pbuf_free(p);
		return; /* pbuf is empty */
	}

	if (input_size == sizeof(input_buffer)) {
		pbuf_free(p);
		return; /* no space */
	}
	if (len > sizeof(input_buffer) - input_size)
		len = sizeof(input_buffer) - input_size;

	end = input_offset + input_size;
	if (end >= sizeof(input_buffer))
		end -= sizeof(input_buffer);

	chunk = len;
	/* Check if packet will wrap in input_buffer */
	if (end + len >= sizeof(input_buffer)) {
		chunk = sizeof(input_buffer) - end;
		/* Copy the second part of the pbuf to start of input_buffer */
		pbuf_copy_partial(p, input_buffer, len - chunk, chunk);
	}
	/* Copy first (or only) part of pbuf after end of current valid input */
	pbuf_copy_partial(p, input_buffer + end, chunk, 0);

	input_size += len;

	/* The client is reachable (it just sent us a packet), but lwIP
	 * does not update the ARP cache from incoming IP frames. If necessary,
	 * initiate an ARP request so the reply arrives before the next
	 * nc_send_packet call needs it. */
	nc_arp_resolve();

	pbuf_free(p);
}

static int nc_ensure_pcb(void)
{
	if (!nc_pcb) {
		nc_pcb = udp_new();
		if (!nc_pcb) {
			nc_teardown();
			return -1;
		}
	}

	if (nc_bound_port != nc_in_port) {
		if (udp_bind(nc_pcb, IP_ADDR_ANY, nc_in_port) != ERR_OK) {
			nc_teardown();
			return -1;
		}
		nc_bound_port = nc_in_port;
		udp_recv(nc_pcb, nc_recv, NULL);
	}

	return 0;
}

static void nc_send_packet(const char *buf, int len)
{
	struct pbuf *p;
	struct netif *netif;

	/* Manual checks since there is not net_loop to do it */
	if (refresh_settings_from_env() < 0)
		return;
	if (nc_ensure_pcb() < 0)
		return;

	netif = nc_get_netif();
	if (!netif)
		return;

	/* Drain the RX ring - an ARP reply may be sitting unprocessed. */
	net_lwip_rx(eth_get_dev(), netif);
	nc_arp_resolve();

	p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
	if (!p)
		return;
	if (pbuf_take(p, buf, len) != ERR_OK) {
		pbuf_free(p);
		return;
	}

	udp_sendto(nc_pcb, p, &nc_ip, nc_out_port);
	pbuf_free(p);
}

static int nc_stdio_start(struct stdio_dev *dev)
{
	int retval;

	nc_out_port = 6666; /* default port */
	nc_in_port = nc_out_port;

	retval = refresh_settings_from_env();
	if (retval != 0)
		return retval;

	return 0;
}

static void nc_stdio_putc(struct stdio_dev *dev, char c)
{
	if (output_recursion)
		return;
	output_recursion = 1;

	nc_send_packet(&c, 1);

	output_recursion = 0;
}

static void nc_stdio_puts(struct stdio_dev *dev, const char *s)
{
	int len;

	if (output_recursion)
		return;
	output_recursion = 1;

	len = strlen(s);
	while (len) {
		int send_len = min(len, (int)sizeof(input_buffer));
		nc_send_packet(s, send_len);
		len -= send_len;
		s += send_len;
	}

	output_recursion = 0;
}

/* lwIP analog of net_loop(NETCONS) */
static int poll_rx(void)
{
	struct netif *netif;

	/* We may have been reached from a network driver wait loop that polls
	 * ctrlc() and must not re-enter the network stack. */
	if (net_lwip_busy())
		return -1;

	/* Manual checks since there is not net_loop to do it */
	if (refresh_settings_from_env() < 0)
		return -1;
	if (nc_ensure_pcb() < 0)
		return -1;

	netif = nc_get_netif();
	if (!netif)
		return -1;

	net_lwip_rx(eth_get_dev(), netif);
	return 0;
}

static int nc_stdio_getc(struct stdio_dev *dev)
{
	uchar c;

	input_recursion = 1;

	while (!input_size) {
		/* net_lwip_rx() schedules; when polling fails before
		 * reaching it, keep the watchdog fed ourselves. */
		if (poll_rx() < 0)
			schedule();
	}

	input_recursion = 0;

	c = input_buffer[input_offset++];

	if (input_offset >= sizeof(input_buffer))
		input_offset -= sizeof(input_buffer);
	input_size--;

	return c;
}

static int nc_stdio_tstc(struct stdio_dev *dev)
{
	/* tstc() is reachable from inside the network stack (for example,
	 * zynq_gem_send's TX-done wait polls ctrlc(), which calls tstc()).
	 * Prevent re-entering the network stack and wedging the MAC. */
	if (input_recursion || output_recursion)
		return 0;

	if (input_size)
		return 1;

	input_recursion = 1;

	poll_rx();	/* kind of poll */

	input_recursion = 0;

	return input_size != 0;
}

int drv_nc_init(void)
{
	struct stdio_dev dev;
	int rc;

	memset(&dev, 0, sizeof(dev));

	strcpy(dev.name, "nc");
	dev.flags = DEV_FLAGS_OUTPUT | DEV_FLAGS_INPUT;
	dev.start = nc_stdio_start;
	dev.putc = nc_stdio_putc;
	dev.puts = nc_stdio_puts;
	dev.getc = nc_stdio_getc;
	dev.tstc = nc_stdio_tstc;

	rc = stdio_register(&dev);

	return (rc == 0) ? 1 : rc;
}
