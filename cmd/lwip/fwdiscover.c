// SPDX-License-Identifier: GPL-2.0+
/*
 * Firmware Discovery Protocol (lwIP implementation)
 *
 * Broadcasts UDP discovery requests to locate firmware servers
 * and retrieves commands to execute.
 */

#include <command.h>
#include <console.h>
#include <dm/device.h>
#include <env.h>
#include <net.h>
#include <lwip/udp.h>
#include <lwip/pbuf.h>
#include <lwip/timeouts.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define FWDISCO_PORT 9875
#define FWDISCO_TIMEOUT 1000  /* 1 second timeout per iteration */
#define FWDISCO_DEFAULT_ITERATIONS 3
#define FWDISCO_MAX_ITERATIONS 10
#define FWDISCO_VERSION "1"
#define FWDISCO_MAX_RESPONSE_LEN 1500

/* State for the discovery protocol */
static struct {
	struct udp_pcb *pcb;		/* UDP protocol control block */
	struct netif *netif;		/* Network interface */
	struct udevice *udev;		/* Ethernet device */
	int iterations_remaining;	/* How many more attempts */
	int total_iterations;		/* Total iterations requested */
	int found;			/* Whether we found a response */
	int done;			/* Whether we're done (success or failure) */
} fwdisco_state;

/*
 * Receive callback - handle incoming UDP packets
 */
static void fwdisco_recv_callback(void *arg, struct udp_pcb *pcb,
				  struct pbuf *p, const ip_addr_t *addr,
				  u16_t port)
{
	char *response;
	char server_ip[40];
	int len;

	printf("Firmware discovery: received packet from %s:%d (%d bytes)\n",
	       ipaddr_ntoa(addr), port, p->len);

	/* Already found a valid response? */
	if (fwdisco_state.found) {
		pbuf_free(p);
		return;
	}

	/* Copy packet data to null-terminated buffer */
	len = p->len;
	if (len >= FWDISCO_MAX_RESPONSE_LEN)
		len = FWDISCO_MAX_RESPONSE_LEN - 1;

	response = malloc(len + 1);
	if (!response) {
		pbuf_free(p);
		return;
	}

	pbuf_copy_partial(p, response, len, 0);
	response[len] = '\0';
	pbuf_free(p);

	/*
	 *   FWRESP/1\n
	 *   ncip : <host ip> \n
	 *   ncinport : <port to listen on> \n
	 *   ncoutport : <port host listens on> \n
	 *   \n
	 *   <body>
	 *   commands are no longer supported in FWRESP
	 */
	if (strncmp(response, "FWRESP/", 7) != 0) {
		free(response);
		return;
	}

	/* Find blank line (header/body separator) */
	char *body = strstr(response, "\n\n");
	if (!body) {
		printf("Firmware discovery: no header/body separator in response\n");
		free(response);
		return;
	}
	body += 2;

	/* NUL-terminate the first line (between "FWRESP/" and first \n) */
	char *eol = strchr(response + 7, '\n');
	if (eol)
		*eol = '\0';

	/* check if the protocol version matches FWDISCO_VERSION */
	char *fwdisco_version = response + 7;
	while (*fwdisco_version == ' ')
		fwdisco_version++;
	char *eov = strchr(fwdisco_version, ' ');
	if (eov)
		*eov = '\0';
	if (strcmp(fwdisco_version, FWDISCO_VERSION) != 0) {
		free(response);
		return;
	}

	/* Store server IP */
	ipaddr_ntoa_r(addr, server_ip, sizeof(server_ip));
	env_set("firmware_server", server_ip);

	printf("Firmware discovery: response from %s (FWRESP version %s)\n",
	       server_ip, fwdisco_version);

	/* Parse header key:value pairs */
	bool got_ncip = false;
	bool got_nc = false;
	char *bol = eol + 1;
	char *colon, *key, *kend, *value, *vend;
	while (bol < body - 1) {
		eol = strchr(bol, '\n');
		if (!eol)
			break;
		*eol = '\0';

		colon = strchr(bol, ':');
		if (!colon)
			continue;
		*colon = '\0';

		/* split key/value at the colon */
		key = bol;
		kend = colon;
		value = colon + 1;
		vend = eol;

		while (*key == ' ')
			key++;
		while (kend > key && (*(kend - 1) == ' '))
			*--kend = '\0';
		while (*value == ' ')
			value++;
		while (vend > value && (*(vend - 1) == ' '))
			*--vend = '\0';

		/* set nc vars if present */
		if (strcmp(key, "ncip") == 0) {
			env_set("ncip", value);
			got_ncip = true;
		} else if (strcmp(key, "ncinport") == 0) {
			env_set("ncinport", value);
		} else if (strcmp(key, "ncoutport") == 0) {
			env_set("ncoutport", value);
		}

		bol = eol + 1;
	}

	if (got_ncip) {
		env_set("stdout", "nc,serial");
		env_set("stderr", "nc,serial");
		env_set("stdin", "nc,serial");
	}

	fwdisco_state.found = 1;
	fwdisco_state.done = 1;

	free(response);
}

/*
 * Send a firmware discovery broadcast request
 */
static void fwdisco_send(void)
{
	struct pbuf *p;
	char *payload;
	int len;
	ip_addr_t broadcast;
	err_t err;
	const char *mfr, *name, *rev, *serial, *bootloader, *bootloader_version;

	/* Prepare broadcast IP */
	IP_ADDR4(&broadcast, 255, 255, 255, 255);

	/* Allocate packet buffer */
	p = pbuf_alloc(PBUF_TRANSPORT, 128, PBUF_RAM);
	if (!p) {
		printf("Firmware discovery: failed to allocate packet\n");
		return;
	}

	/* Build request packet: "FWREQ <version> <identifier> <serial>" */
	mfr = env_get("board_manufacturer");
	name = env_get("board_name");
	rev = env_get("board_rev");
	serial = env_get("board_serial");
	bootloader = env_get("bootloader");
	bootloader_version = env_get("bootloader_version");

	payload = (char *)p->payload;
	len = snprintf(payload, 128,
		       "FWREQ/%s\nManufacturer: %s\nProduct: %s\nRevision: %s\nSerial: %s\nBootloader: %s\nBootloader Version: %s\n\n",
		       FWDISCO_VERSION,
		       mfr ? mfr : "unknown",
		       name ? name : "unknown",
		       rev ? rev : "?",
		       serial ? serial : "unknown",
		       bootloader ? bootloader : "unknown",
		       bootloader_version ? bootloader_version : "unknown");

	pbuf_realloc(p, len);

	/* Send UDP broadcast */
	err = udp_sendto(fwdisco_state.pcb, p, &broadcast, FWDISCO_PORT);
	pbuf_free(p);

	if (err != ERR_OK) {
		printf("Firmware discovery: failed to send broadcast (err=%d)\n", err);
		return;
	}

	if (fwdisco_state.total_iterations == 0)
		printf("Firmware discovery: broadcasting request...\n");
	else
		printf("Firmware discovery: broadcasting request (try %d/%d)...\n",
		       (fwdisco_state.total_iterations - fwdisco_state.iterations_remaining + 1),
		       fwdisco_state.total_iterations);
}

/*
 * Timeout handler - retry or fail
 */
static void fwdisco_timeout_handler(void *arg)
{
	if (fwdisco_state.total_iterations == 0) {
		/* Run forever — just retry */
		sys_timeout(FWDISCO_TIMEOUT, fwdisco_timeout_handler, NULL);
		fwdisco_send();
		return;
	}

	fwdisco_state.iterations_remaining--;

	if (fwdisco_state.iterations_remaining > 0) {
		/* Try again */
		sys_timeout(FWDISCO_TIMEOUT, fwdisco_timeout_handler, NULL);
		fwdisco_send();
	} else {
		/* All attempts exhausted */
		puts("Firmware discovery: timeout - no response received\n");
		fwdisco_state.done = 1;
	}
}

/*
 * Main discovery loop
 */
static int fwdisco_loop(struct udevice *udev, int iterations)
{
	struct netif *netif;
	err_t err;
	u16_t our_port;

	/* Initialize network interface */
	netif = net_lwip_new_netif(udev);
	if (!netif) {
		printf("Firmware discovery: failed to initialize network interface\n");
		return -1;
	}

	/* Check we have a valid IP address */
	if (ip_addr_isany(&netif->ip_addr)) {
		printf("Firmware discovery: ERROR - no IP address set (run 'dhcp' first)\n");
		net_lwip_remove_netif(netif);
		return -1;
	}

	/* Initialize state */
	memset(&fwdisco_state, 0, sizeof(fwdisco_state));
	fwdisco_state.netif = netif;
	fwdisco_state.udev = udev;
	fwdisco_state.iterations_remaining = iterations;
	fwdisco_state.total_iterations = iterations;

	/* Create UDP PCB */
	fwdisco_state.pcb = udp_new();
	if (!fwdisco_state.pcb) {
		printf("Firmware discovery: failed to create UDP socket\n");
		net_lwip_remove_netif(netif);
		return -1;
	}

	/* Bind to random port */
	our_port = 10000 + (get_timer(0) % 4096);
	err = udp_bind(fwdisco_state.pcb, IP_ADDR_ANY, our_port);
	if (err != ERR_OK) {
		printf("Firmware discovery: failed to bind UDP socket (err=%d)\n", err);
		udp_remove(fwdisco_state.pcb);
		net_lwip_remove_netif(netif);
		return -1;
	}

	/* Set receive callback */
	udp_recv(fwdisco_state.pcb, fwdisco_recv_callback, NULL);

	/* Enable broadcast on the PCB */
	ip_set_option(fwdisco_state.pcb, SOF_BROADCAST);

	/* Send first request */
	fwdisco_send();

	/* Set timeout */
	sys_timeout(FWDISCO_TIMEOUT, fwdisco_timeout_handler, NULL);

	/* Main loop */
	while (!fwdisco_state.done) {
		net_lwip_rx(udev, netif);
		if (ctrlc()) {
			printf("\nAborted by user\n");
			break;
		}
	}

	/* Cleanup */
	sys_untimeout(fwdisco_timeout_handler, NULL);
	udp_remove(fwdisco_state.pcb);
	net_lwip_remove_netif(netif);

	if (fwdisco_state.found)
		return 0;

	/* Clear environment variables on failure */
	env_set("firmware_boot_cmd", NULL);
	env_set("firmware_version", NULL);
	return -1;
}

/*
 * Command implementation
 */
static int do_fwdiscover(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	int iterations = FWDISCO_DEFAULT_ITERATIONS;

	if (argc > 2)
		return CMD_RET_USAGE;

	if (argc >= 2) {
		iterations = simple_strtoul(argv[1], NULL, 10);
		if (iterations > FWDISCO_MAX_ITERATIONS) {
			printf("Invalid iteration count (must be 0-%d, 0=forever)\n",
			       FWDISCO_MAX_ITERATIONS);
			return CMD_RET_USAGE;
		}
	}

	/* Start ethernet */
	if (net_lwip_eth_start() < 0)
		return CMD_RET_FAILURE;

	/* Run discovery loop */
	if (fwdisco_loop(eth_get_dev(), iterations) < 0)
		return CMD_RET_FAILURE;

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	fwdiscover, 2, 0, do_fwdiscover,
	"discover firmware server via UDP broadcast",
	"[iterations]\n"
	"    - Send UDP broadcast requests to discover firmware server\n"
	"    - iterations: number of attempts (default: 3, max: 10, 0=forever)\n"
	"    - Identifier built from $board_manufacturer, $board_name, $board_rev\n"
	"    - Serial from $board_serial sent for server-side filtering\n"
	"    - Sets environment variables on success:\n"
	"        firmware_server: IP address of responding server\n"
	"        ncip: IP address for netconsole to send packets to\n"
	"        ncinport: port netconsole listens on\n"
	"        ncoutport: destination port netconsole will send packets to\n"
	"    - Example: fwdiscover 5"
);
